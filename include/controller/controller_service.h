#pragma once
#include <tcp_messages.h>
#include <cell_world.h>
#include <controller/pid_controller.h>
#include <controller/agent.h>
#include <agent_tracking/tracking_client.h>
#include <experiment/experiment_client.h>
#include <controller/controller_client.h>

namespace controller {

    enum Controller_state{
        Stopped,
        Playing,
        Paused
    };

    struct Controller_service : tcp_messages::Message_service {
        Routes (
                Add_route_with_response("set_destination", set_destination, cell_world::Location);
                Add_route_with_response("stop", stop_controller);
                Add_route_with_response("arm", arm);
                Add_route_with_response("pause", pause_controller);
                Add_route_with_response("resume", resume_controller);
                Add_route_with_response("set_behavior", set_behavior, int);
                Allow_subscription();
        );

        bool set_destination(const cell_world::Location &);
        bool stop_controller();
        bool arm();
        bool pause_controller();
        bool resume_controller();
        bool set_behavior(int);
        static int get_port();
        static void set_logs_folder(const std::string &);
    };


    struct Agent_data{
        Agent_data (const std::string &agent_name) :
                agent_name(agent_name) {
            step.agent_name = agent_name;
        }
        std::string agent_name;
        cell_world::Step step;
        cell_world::Timer timer;
        cell_world::Timer last_update;
        bool is_valid() {
            return !timer.time_out();
        }
    };

    struct Controller_server : tcp_messages::Message_server<Controller_service> {
        void send_step(const cell_world::Step &);
        void send_capture(int);
        bool set_destination(const cell_world::Location &);
        bool pause();
        bool resume();
        void set_occlusions(const std::string &occlusions, float margin = .45);
        bool set_behavior(int behavior);
        void join();

        struct Controller_experiment_client : experiment::Experiment_client {
            explicit Controller_experiment_client();
            void on_experiment_started(const experiment::Start_experiment_response &experiment) override;
            void on_episode_started(const std::string &experiment_name) override;
            Controller_server *controller_server;
        } &experiment_client;

        struct Controller_tracking_client : agent_tracking::Tracking_client {
            Controller_tracking_client(cell_world::Location_visibility &visibility,
                                       float view_angle,
                                       cell_world::Capture &capture,
                                       cell_world::Peeking &peeking,
                                       const std::string &agent_name,
                                       const std::string &adversary_name);
            Controller_tracking_client(cell_world::World,
                                       float view_angle,
                                       const std::string &agent_name,
                                       const std::string &adversary_name);
            void on_step(const cell_world::Step &step) override;
            void set_occlusions(cell_world::Cell_group &);
            Agent_data agent;
            Agent_data adversary;
            cell_world::Location_visibility visibility;
            Controller_server *controller_server;
            float view_angle;
            cell_world::Capture capture;
            cell_world::Peeking peeking;
        } &tracking_client;
        Controller_server(const std::string &pid_config_file_path,
                          Agent &,
                          Controller_tracking_client &,
                          Controller_experiment_client &,
                          bool);

        Controller_server(const std::string &pid_config_file_path,
                          Agent &,
                          Controller_tracking_client &,
                          Controller_experiment_client &,
                          cell_world::Location &robot_destination,
                          cell_world::Location &robot_normalized_destination,
                          cell_world::Location &gravity_adjustment,
                          bool);


        void controller_process();
        void arm();
        cell_world::Location get_next_stop();

        template< typename T, typename... Ts>
        T &create_local_client(Ts... vs){
            static_assert(std::is_base_of<Controller_client, T>::value, "T must inherit from Controller_client");
            auto new_local_client = new T{ vs... };
            local_clients.push_back((Controller_client *) new_local_client);
            new_local_client->local_server = this;
            return *new_local_client;
        }

        bool subscribe_local( Controller_client *client) {
            subscribed_local_clients.push_back(client);
            return true;
        }

        bool unsubscribe_local(Controller_client *client) {
            subscribed_local_clients.erase(std::remove(subscribed_local_clients.begin(), subscribed_local_clients.end(), client));
            return true;
        }

        bool remove_local_client(Controller_client *client) {
            subscribed_local_clients.erase(std::remove(subscribed_local_clients.begin(), subscribed_local_clients.end(), client));
            local_clients.erase(std::remove(local_clients.begin(), local_clients.end(), client));
            delete client;
            return true;
        }

        std::vector<Controller_client * > local_clients;
        std::vector<Controller_client * > subscribed_local_clients;


        void set_occlusions(cell_world::Cell_group &);

        cell_world::Location destination;
        cell_world::Timer destination_timer;
        bool new_destination_data;
        std::atomic<Controller_state> state;
        Agent &agent;
        Behavior behavior = Explore;
        cell_world::World world;
        cell_world::World world_paths;
        cell_world::Cell_group cells;
        cell_world::Cell_group free_cells;
        cell_world::Paths paths;
        cell_world::Map map;


        cell_world::Location_visibility navigability;
        Pid_controller pid_controller;
        std::thread process;
        cell_world::Location &robot_destination;
        cell_world::Location &robot_normalized_destination;
        cell_world::Location &gravity_adjustment;
        cell_world::Location local_robot_destination;
        cell_world::Location local_robot_normalized_destination;
        cell_world::Location local_gravity_adjustment;
        bool armed;
        bool manually_armed;

    };
}