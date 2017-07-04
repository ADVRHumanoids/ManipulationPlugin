/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Luca Muratore
 * email: luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <ManipulationPlugin_rt_plugin.h>

/* Specify that the class XBotPlugin::ManipulationPlugin is a XBot RT plugin with name "ManipulationPlugin" */
REGISTER_XBOT_PLUGIN(ManipulationPlugin, XBotPlugin::ManipulationPlugin)

namespace XBotPlugin {

bool ManipulationPlugin::init_control_plugin(std::string path_to_config_file,
                                                    XBot::SharedMemory::Ptr shared_memory,
                                                    XBot::RobotInterface::Ptr robot)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = robot;

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */
    
    _logger = XBot::MatLogger::getLogger("/tmp/ManipulationPlugin_log");


    /*Saves robot as shared variable between states*/
    fsm.shared_data()._robot= robot;

    /*Registers states*/
    fsm.register_state(std::make_shared<myfsm::Ready>());
    fsm.register_state(std::make_shared<myfsm::Move>());
    
    // Initialize the FSM with the initial state
    fsm.init("Ready");
    
    
    
    return true;


}

void ManipulationPlugin::on_start(double time)
{

    /* Save the plugin starting time to a class member */
    _robot->getMotorPosition(_q0);

    /* Save the robot starting config to a class member */
    _start_time = time;
}

void ManipulationPlugin::on_stop(double time)
{
}


void ManipulationPlugin::control_loop(double time, double period)
{
    // run the FSM
    fsm.run(time, 0.005);

}

bool ManipulationPlugin::close()
{
    /* Save logged data to disk */
    _logger->flush();

    return true;
}



}
