#include <controller/controller_service.h>
#include <filesystem>
#include <iostream>
#include <mutex>

using namespace cell_world;
using namespace tcp_messages;
using namespace std;
using namespace json_cpp;

namespace controller {
    mutex robot_mtx;
    string logs_path = "";

    bool Controller_service::set_destination(const cell_world::Location &location) {
        return ((Controller_server *) _server)->set_destination(location);
    }

    bool Controller_service::stop_controller() {
        return true;
    }

    bool Controller_service::pause_controller() {
        return ((Controller_server *) _server)->pause();
    }

    bool Controller_service::resume_controller() {
        return ((Controller_server *) _server)->resume();
    }

    int Controller_service::get_port() {
        string port_str(std::getenv("CONTROLLER_PORT") ? std::getenv("CONTROLLER_PORT") : "4590");
        return atoi(port_str.c_str());
    }

    bool Controller_service::set_behavior(int behavior) {
        return ((Controller_server *) _server)->set_behavior(behavior);
    }

    void Controller_service::set_logs_folder(const string &path) {
        logs_path = path;
        filesystem::create_directory(filesystem::path(logs_path));
    }

    void Controller_server::send_step(const Step &step) {
        if (!clients.empty()) {
            broadcast_subscribed(Message (step.agent_name + "_step", step));
        }
        for (auto local_client: subscribed_local_clients){
            local_client->on_step(step);
        }
    }



    Controller_server::Controller_server(const string &pid_config_file_path,
                                         Agent &agent,
                                         Controller_tracking_client &tracking_client,
                                         Controller_experiment_client &experiment_client,
                                         Location &robot_destination,
                                         Location &gravity_adjustment):
            agent(agent),
            world(World::get_from_parameters_name("hexagonal", "canonical")),
            world_paths(World::get_from_parameters_name("hexagonal", "canonical")),
            cells(world.create_cell_group()),
            paths(world.create_paths(Resources::from("paths").key("hexagonal").key("00_00").key("astar").get_resource<Path_builder>())),
            map(cells),
            navigability(cells, world.cell_shape, world.cell_transformation),
            pid_controller(Json_from_file<Pid_parameters>(pid_config_file_path)),
            tracking_client(tracking_client),
            destination_timer(0),
            experiment_client(experiment_client),
            robot_destination(robot_destination),
            gravity_adjustment(gravity_adjustment)
    {
        tracking_client.controller_server = this;
        experiment_client.controller_server = this;
        experiment_client.subscribe();
        tracking_client.subscribe();
        state = Controller_state::Stopped;
        process = thread(&Controller_server::controller_process, this);
    }

#define progress_translation 0.003375
#define progress_rotation 1.25
#define progress_time 0.25

    Location progress_marker_translation;
    float progress_marker_rotation;
    Timer progress_timer(progress_time);

    void Controller_server::controller_process() {                      // setting robot velocity
        state = Controller_state::Playing;
        Pid_inputs pi;
        Timer msg(1);
        bool human_intervention = false;
        while(state != Controller_state::Stopped){
            robot_mtx.lock();
            if (this->tracking_client.capture.cool_down.time_out()){
            // if there is no information from the tracker
                if (!tracking_client.agent.is_valid() ||
                    state == Controller_state::Paused ||
                    destination_timer.time_out()){
                    agent.set_left(0);
                    agent.set_right(0);
                    agent.update();
                    progress_timer.reset();
                } else {
                    //PID controller
                    pi.location = tracking_client.agent.step.location;
                    pi.rotation = tracking_client.agent.step.rotation;
                    auto theta_diff = to_degrees(angle_difference(to_radians(progress_marker_rotation),to_radians(pi.rotation)));
//                    if (msg.time_out()) {
//                        msg.reset();
//                        cout << "progress_marker: " << progress_marker << endl;
//                        cout << "location: " << pi.location << endl;
//                        cout << "distance: " << pi.location.dist(progress_marker) << endl;
//                        cout << "progress_marker_theta: " << progress_marker_theta << endl;
//                        cout << "rotation: " << pi.rotation << endl;
//                        cout << "diff: " << theta_diff << endl;
//                        cout << "time since progress:" << progress_timer.to_seconds() << endl;
//                    }
                    if (pi.location.dist(progress_marker_translation) > progress_translation ||
                            theta_diff > progress_rotation) {
                        progress_marker_rotation = pi.rotation;
                        progress_marker_translation = pi.location;
                        progress_timer.reset();
                    }
                    if (progress_timer.time_out()){
                        cout << "I think I am stuck - backing up" << endl;
                        progress_timer.reset();
                        agent.set_left(-20);
                        agent.set_right(-20);
                        agent.update();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        pi.location = tracking_client.agent.step.location;
                        pi.rotation = tracking_client.agent.step.rotation;
                        progress_marker_rotation = pi.rotation;
                        progress_marker_translation = pi.location;
                    } else {
                        pi.destination = get_next_stop();
                        auto dist = destination.dist(pi.location);
                        if (dist < world.cell_transformation.size / 2) {
                            agent.set_left(0);
                            agent.set_right(0);
                            agent.update();
                        } else {
                            auto robot_command = pid_controller.process(pi, behavior);
                            agent.set_left(robot_command.left);
                            agent.set_right(robot_command.right);
                            if (agent.human_intervention!=human_intervention){
                                agent.human_intervention = human_intervention;
                                experiment_client.human_intervention(agent.human_intervention);
                            }
                            agent.update();
                        }
                    }
                }
            }
            robot_mtx.unlock();
            //prevents overflowing the robot ( max 10 commands per second)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    bool Controller_server::set_destination(const cell_world::Location &new_destination) {
        //cout << "New destination: " << new_destination << endl;
        destination = new_destination;
        destination_timer = Timer(5);
        new_destination_data = true;
        return true;
    }

#define goal_weight 0.00000005
#define occlusion_weight 0.000005 //0.0001 //.0001 //was 0.0001
#define decay 3 //2 //5 //2
#define gravity_threshold .3
#define gravity_change_limit .05


    cell_world::Location Controller_server::get_next_stop() {
        auto agent_location = tracking_client.agent.step.location;
        auto destination_cell_index = cells.find(destination);
        auto next_stop_test = cells.find(agent_location);
        auto next_stop = next_stop_test;
        if (navigability.is_visible(agent_location, destination)) {
            next_stop = destination_cell_index;
        } else {
            while (navigability.is_visible(agent_location, cells[next_stop_test].location)) {
                next_stop = next_stop_test;
                auto move = paths.get_move(cells[next_stop], cells[destination_cell_index]);
                if (move == Move{0, 0}) break;
                next_stop_test = cells.find(map[cells[next_stop].coordinates + move]);
            }
        }

        auto total_gravity_change = Location(0,0);
        for (auto &cell_r : cells.occluded_cells()) {
            Cell cell = cell_r.get();
            auto distance = cell.location.dist(agent_location);
            if (distance > gravity_threshold) continue;
            auto theta = cell.location.atan(agent_location);
            auto gravity = occlusion_weight / pow(distance,decay);
            total_gravity_change = total_gravity_change.move(theta, gravity);
        }
        auto goal_distance = cells[next_stop].location.dist(agent_location);
        auto theta = agent_location.atan(cells[next_stop].location);
        auto gravity = goal_weight / pow(goal_distance,decay);
        total_gravity_change = total_gravity_change.move(theta, gravity);
        if (total_gravity_change.mod()> gravity_change_limit) {
            total_gravity_change = total_gravity_change / total_gravity_change.mod() * gravity_change_limit;
        }

        robot_destination = cells[next_stop].location;
        gravity_adjustment = total_gravity_change;
        return cells[next_stop].location + total_gravity_change;
    }

    bool Controller_server::pause() {
        if (state == Controller_state::Playing) {
            state = Controller_state::Paused;
            return true;
        }
        return false;
    }

    bool Controller_server::resume() {
        if (state == Controller_state::Paused) {
            state = Controller_state::Playing;
            return true;
        }
        return false;
    }

    void Controller_server::set_occlusions(const std::string &occlusions, float margin) {
        auto occlusions_cgb = Resources::from("cell_group").key("hexagonal").key(occlusions).key("occlusions").get_resource<Cell_group_builder>();
        world.set_occlusions(occlusions_cgb);
        cells = world.create_cell_group();
        auto occlusions_cgb_paths = Resources::from("cell_group").key("hexagonal").key(occlusions).key("occlusions").key("robot").get_resource<Cell_group_builder>();
        world_paths.set_occlusions(occlusions_cgb_paths);
        auto occlusions_path = world_paths.create_cell_group(occlusions_cgb_paths);
        paths = Paths(world.create_paths(Resources::from("paths").key("hexagonal").key(occlusions).key("astar").key("robot").get_resource<Path_builder>()));
        navigability = Location_visibility(occlusions_path, world.cell_shape,Transformation(world.cell_transformation.size * (1 + margin), world.cell_transformation.rotation)); // robot size
        tracking_client.visibility.update_occlusions(cells);
    }

    void Controller_server::join() {
        process.join();
        Server::join();
    }

    bool Controller_server::set_behavior(int behavior) {
        this->behavior = static_cast<Behavior> (behavior);
        experiment_client.set_behavior(behavior);
        return true;
    }

    void Controller_server::send_capture(int frame) {
        experiment_client.capture(frame);
    }

    void Controller_server::set_occlusions(Cell_group &cells) {
        navigability.update_occlusions(cells);
    }

    void Controller_server::Controller_tracking_client::on_step(const Step &step) {
        if (!capture.cool_down.time_out()) return;
        if (step.agent_name == agent.agent_name) {
            if (agent.last_update.to_seconds()>.1) {
                controller_server->send_step(step);
                agent.last_update.reset();
            }
            agent.step = step;
            agent.timer = Timer(.5);
        } else if (step.agent_name == adversary.agent_name) {
            adversary.step = step;
            adversary.timer = Timer(.5);
            if (contains_agent_state(agent.agent_name)) {
                auto predator = get_current_state(agent.agent_name);
                robot_mtx.lock();
                    auto is_captured = capture.is_captured( predator.location, to_radians(predator.rotation), step.location);
                    if (is_captured) {
                        controller_server->agent.set_left(0);
                        controller_server->agent.set_right(0);
                        controller_server->agent.capture();
                        controller_server->agent.update();
                        controller_server->agent.capture();
                        controller_server->agent.update();
                        controller_server->agent.update();
                        controller_server->send_capture(step.frame);
                    }
                robot_mtx.unlock();
                if (visibility.is_visible(predator.location, step.location) &&
                        to_degrees(angle_difference(predator.location.atan(step.location), to_radians(predator.rotation))) < view_angle / 2) {
                    if (peeking.is_seen(predator.location, step.location)) {
                        if (adversary.last_update.to_seconds()>.1) {
                            controller_server->send_step(step);
                            adversary.last_update.reset();
                        }
                    }
                } else {
                    peeking.not_visible();
                }
            }
        }
        Tracking_client::on_step(step);
    }

    Controller_server::Controller_tracking_client::Controller_tracking_client(Location_visibility &visibility,
                                                                              float view_angle,
                                                                              Capture &capture,
                                                                              Peeking &peeking,
                                                                              const string &agent_name,
                                                                              const string &adversary_name)  :
            agent(agent_name),
            adversary(adversary_name),
            visibility(visibility),
            view_angle(view_angle),
            capture(capture),
            peeking(peeking){
    }

    Controller_server::Controller_tracking_client::Controller_tracking_client(cell_world::World world,
                                                                              float view_angle,
                                                                              const string &agent_name,
                                                                              const string &adversary_name):
            agent(agent_name),
            adversary(adversary_name),
            visibility(world.create_cell_group(), world.cell_shape, world.cell_transformation),
            view_angle(view_angle),
            capture(Resources::from("capture_parameters").key("default").get_resource<Capture_parameters>(), world),
            peeking(Resources::from("peeking_parameters").key("default").get_resource<Peeking_parameters>(), world){

    }

    void Controller_server::Controller_tracking_client::set_occlusions(Cell_group &cells) {
        visibility.update_occlusions(cells);
        capture.visibility.update_occlusions(cells);
        peeking.peeking_visibility.update_occlusions(cells);
    }

    string get_experiment_file(const string &experiment_name){
        return logs_path + experiment_name + "_controller.json";
    }

    void Controller_server::Controller_experiment_client::on_experiment_started(
            const experiment::Start_experiment_response &experiment) {
        experiment.save(get_experiment_file(experiment.experiment_name));
        Experiment_client::on_experiment_started(experiment);
    }

    void Controller_server::Controller_experiment_client::on_episode_started(const string &experiment_name) {
        experiment::Start_experiment_response experiment;
        experiment.load(get_experiment_file(experiment_name));
        controller_server->set_occlusions(experiment.world.occlusions);
        Experiment_client::on_episode_started(experiment_name);
    }

    Controller_server::Controller_experiment_client::Controller_experiment_client() {
    }
}

