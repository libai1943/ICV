#include "collab_planner/KeyboardInput.h"
#include "ros/ros.h"

bool handle_keyboard_input(collab_planner::KeyboardInput::Request &req, collab_planner::KeyboardInput::Response &res) {
    ROS_INFO("Received keyboard input: %s", req.data.c_str());
    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_input_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("keyboard_input", handle_keyboard_input);
    ROS_INFO("Ready to receive keyboard input.");
    ros::spin();

    return 0;
}
