#include <ros/ros.h>

#include <iostream>

#include "collab_planner/KeyboardInput.h"

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "keyboard_input_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<collab_planner::KeyboardInput>("keyboard_input");
    collab_planner::KeyboardInput srv;

    while (ros::ok()) {
        std::string input;
        std::cout << "Enter a character: ";
        std::getline(std::cin, input);

        if (input == "exit") {
            break;
        }

        srv.request.data = input;

        if (!client.call(srv)) {
            ROS_ERROR("Failed to call service keyboard_input");
        }
    }

    return 0;
}
