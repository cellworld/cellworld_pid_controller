#pragma once
#include <tcp_messages.h>
#include <cell_world.h>
#include <controller/pid_controller.h>

namespace controller{
    struct Controller_server;

    struct Controller_client : tcp_messages::Message_client {
        Routes(
                Add_route("(.*)(step)", on_step, cell_world::Step);
                )
        virtual void on_step(const cell_world::Step &) {}
        bool pause();
        bool resume();
        bool stop();
        bool set_destination(const cell_world::Location &);
        bool set_behavior(Behavior);
        Controller_server *local_server = nullptr;
    };
}