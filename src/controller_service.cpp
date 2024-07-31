#include <controller/controller_service.h>
#include <filesystem>
#include <iostream>
#include <mutex>
#define NO_ROTATION -1000

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

    bool Controller_service::set_destination_with_rotation(const controller::Destination_with_rotation &destination) {
        return ((Controller_server *) _server)->set_destination_with_rotation(destination);
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
                                         Controller_experiment_client &experiment_client):
            Controller_server(pid_config_file_path,
                              agent,
                              tracking_client,
                              experiment_client,
                              local_robot_destination,
                              local_robot_normalized_destination,
                              local_gravity_adjustment) {}

    Controller_server::Controller_server(const string &pid_config_file_path,
                                         Agent &agent,
                                         Controller_tracking_client &tracking_client,
                                         Controller_experiment_client &experiment_client,
                                         Location &robot_destination,
                                         Location &robot_normalized_destination,
                                         Location &gravity_adjustment):
            agent(agent),
            world(World::get_from_parameters_name("hexagonal", "canonical")),  // delete specified occludiond
            world_paths(World::get_from_parameters_name("hexagonal", "canonical")),
            cells(world.create_cell_group()),
            free_cells(world_paths.create_cell_group().free_cells()),
            paths(world.create_paths(Resources::from("paths").key("hexagonal").key("00_00").key("astar").get_resource<Path_builder>())),  // CHANGE BACK TO 00
            map(cells),
            navigability(cells, world.cell_shape, world.cell_transformation),
            pid_controller(Json_from_file<Pid_parameters>(pid_config_file_path)),
            tracking_client(tracking_client),
            destination_timer(0),
            experiment_client(experiment_client),
            robot_destination(robot_destination),
            robot_normalized_destination(robot_normalized_destination),
            gravity_adjustment(gravity_adjustment),
            destination_rotation(NO_ROTATION)
    {
        tracking_client.controller_server = this;
        experiment_client.controller_server = this;
        experiment_client.subscribe();
        tracking_client.subscribe();
        state = Controller_state::Stopped;
        process = thread(&Controller_server::controller_process, this);
    }

#define progress_translation 0.00675
#define progress_rotation 2.5
#define progress_time 0.5  // changed this for open field was 0.5  // open field 3.0

    Location progress_marker_translation;
    float progress_marker_rotation;
    Timer progress_timer(progress_time);

    void Controller_server::controller_process() {                      // setting robot velocity
//        set_occlusions("030_12_0063"); // DELETE ONCE EXPERIMENT SERVER ON
        state = Controller_state::Playing;
        Pid_inputs pi;
        Timer msg(1);
        bool human_intervention = false;
        bool manual_enter = true;
        while(state != Controller_state::Stopped){
            robot_mtx.lock();

            // if there is no information from the tracker robot wont move
            if (this->tracking_client.capture.cool_down.time_out()){
                if (!tracking_client.agent.is_valid() ||  // leds will turn off when not connects
                    state == Controller_state::Paused ||
                    agent.human_intervention ||
                    destination_timer.time_out()){
                    agent.set_left(0);
                    agent.set_right(0);
                    agent.update();
                    progress_timer.reset();
                    if (agent.human_intervention && manual_enter) {
                        cout << "MANUAL" << endl;
                        manual_enter = false;
                    }
                } else { //PID controller
                    manual_enter = true;
                    pi.location = tracking_client.agent.step.location;
                    pi.rotation = tracking_client.agent.step.rotation;
                    auto theta_diff = to_degrees(angle_difference(to_radians(progress_marker_rotation),to_radians(pi.rotation)));

                    // AUTO BACKUP
                    if (pi.location.dist(progress_marker_translation) > progress_translation ||
                            theta_diff > progress_rotation) {
                        progress_marker_rotation = pi.rotation;
                        progress_marker_translation = pi.location;
                        progress_timer.reset();
                    }
                    if (false){  // BACKUP I DONT WANT THIS (progress_timer.time_out() && destination.dist(pi.location) > .1)
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

                        // slow rotation if close enough to ambush cell
                        if (dist <= world.cell_transformation.size * 0.5 and destination_rotation != NO_ROTATION) {  // for open field
                            cout << "CLOSE ENOUGH TO AMBUSH CELL SLOW ROTATION" << endl;
                            // TODO: may need to normalize angle
                            auto destination_theta = to_radians(destination_rotation);
                            auto theta = to_radians(pi.rotation);
                            auto error = angle_difference(theta, destination_theta);
                            cout << "DIRECTION: " << direction(theta, destination_theta) << " ERROR: " << error << endl;
                            cout << "DESTINATION ANGLE: " << destination_theta << " ACTUAL ANGLE" << pi.rotation << endl;
                            if (error > 5.0){
                                // spin slowly
                                agent.set_left(10 * direction(theta, destination_theta)); // TODO: check this
                                agent.set_right(-10 * direction(theta, destination_theta));
                                if (direction(theta, destination_theta) > 0.0) cout << "SPIN CW" << endl;
                                agent.update();
                            } else {
                                agent.set_left(0); // TODO: check this
                                agent.set_right(0);
                                agent.update();
                            }
                        // VANILLA PID
                        } else {
                            auto robot_command = pid_controller.process(pi, behavior);
                            //cout << robot_command.left << " " << robot_command.right << endl;
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
//            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    bool Controller_server::set_destination(const cell_world::Location &new_destination) {
        //cout << "New destination: " << new_destination << endl;
        destination = new_destination;
        destination_timer = Timer(5);
        new_destination_data = true;
        destination_rotation = NO_ROTATION;
        progress_timer.reset();
        return true;
    }

    bool Controller_server::set_destination_with_rotation(const Destination_with_rotation &new_destination) {
        destination = new_destination.location;
        destination_timer = Timer(5);
        new_destination_data = true;
        destination_rotation = new_destination.rotation;
        progress_timer.reset();
        return true;
    }


#define goal_weight 0.0
    #define occlusion_weight 0.0075 //0.0015
    #define decay 2 //2 //5 //2
    #define gravity_threshold .15


    double normalize_error(double error){
        double pi_err =  M_PI * error / 2;
        return 1 / ( (pi_err * pi_err) + 1 );
    }

    cell_world::Location Controller_server::get_next_stop() {
        auto agent_location = tracking_client.agent.step.location;
        auto agent_theta = to_radians(tracking_client.agent.step.rotation);
        auto destination_cell_index = free_cells.find(destination);


        auto agent_cell_id = free_cells.find(agent_location);
        auto next_stop_test = agent_cell_id;
        auto next_stop = agent_cell_id;
        if (navigability.is_visible(agent_location, destination)) {
            next_stop = destination_cell_index;
        } else {
            while (navigability.is_visible(agent_location, free_cells[next_stop_test].location)) {
                next_stop = next_stop_test;
                auto move = paths.get_move(free_cells[next_stop], free_cells[destination_cell_index]);

                if (move == Move{0, 0}) break;
                next_stop_test = free_cells.find(map[free_cells[next_stop].coordinates + move]);
            }
        }
        robot_destination = free_cells[next_stop].location;
        auto destination_theta = agent_location.atan(robot_destination);
        auto translation = robot_destination - agent_location;
        auto translation_magnitude = translation.mod();
        auto normalized_translation = translation / translation_magnitude;
        robot_normalized_destination = agent_location + normalized_translation * world.cell_transformation.size;

        float total_gravity = 0;
        for (auto &cell_r : cells.occluded_cells()) {
            Cell cell = cell_r.get();
            auto distance = cell.location.dist(agent_location);
            if (distance > gravity_threshold) continue;
            auto occlusion_theta = agent_location.atan(cell.location);
            auto err = angle_difference(occlusion_theta,agent_theta);
            if (err > M_PI / 2) continue;
            auto normalized_error = normalize_error(err);
            auto occlusion_direction = direction(destination_theta, occlusion_theta);
            auto occlusion_gravity = normalized_error * occlusion_weight / pow(distance,decay);
            total_gravity += occlusion_gravity * occlusion_direction;
//            cout << occlusion_gravity << "," << occlusion_direction << endl;
        }
        if (total_gravity > 1) total_gravity = 1;
        auto total_gravity_change = Location(0,0).move(destination_theta + M_PI / 2, total_gravity);
        gravity_adjustment = total_gravity_change * world.cell_transformation.size;
        auto normalized_updated_translation = normalized_translation + total_gravity_change;
        auto updated_translation = normalized_updated_translation * translation_magnitude;
        return agent_location + updated_translation;
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
        free_cells = world_paths.create_cell_group().free_cells();
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
        } else if ((step.agent_name == adversary.agent_name) || (step.agent_name.starts_with("mouse"))) {
            adversary.step = step;
            adversary.timer = Timer(.5);
            if (contains_agent_state(agent.agent_name)) {
                auto predator = get_current_state(agent.agent_name);

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

                robot_mtx.lock();
                    auto is_captured = capture.is_captured( predator.location, to_radians(predator.rotation), step.location);
                    //controller_server.need_capture = true;
                    if (is_captured) {
                        controller_server->agent.set_left(0);
                        controller_server->agent.set_right(0);
                        controller_server->agent.capture();
                        controller_server->agent.update();
                        //controller_server->agent.capture();
                        //controller_server->agent.update();
                        controller_server->agent.end_capture();
                        controller_server->agent.update();
                        controller_server->send_capture(step.frame);
                    }
                robot_mtx.unlock();
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

