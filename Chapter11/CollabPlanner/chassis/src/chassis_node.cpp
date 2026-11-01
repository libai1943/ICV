#include <TinyProtocol.h>
#include <geometry_msgs/PoseStamped.h>
#include <hal/tiny_serial.h>
#include <ros/ros.h>
#include <sandbox_msgs/ChassisCommand.h>
#include <signal.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <atomic>
#include <functional>
#include <thread>

ros::Publisher *velocity_publisher_;
ros::Subscriber *goal_sub_;

double max_phi_plan_ = 0.55;
double min_phi_plan_ = -0.55;
double steering_rad_ = 0.0;

#define MSG_TYPE_TWIST 0xe1
#define MSG_TYPE_PID 0xe2
#define MSG_TYPE_VELOCITY 0xe3

struct TwistMessage {
    int16_t velocity;
    int16_t angle;
};

struct PIDMessage {
    float kp;
    float ki;
};

struct VelocityMessage {
    int16_t velocity;
};

typedef unsigned char byte;

void HandleMessage(byte type, byte *data, int len) {
    if (type == MSG_TYPE_VELOCITY) {
        if (len != sizeof(VelocityMessage)) return;
        VelocityMessage vel{};
        std::memcpy(&vel, data, sizeof(vel));

        std_msgs::Float64 msg;
        msg.data = vel.velocity / 100.0;
        velocity_publisher_->publish(msg);
    }
}

int16_t GenerateMessage(byte target, byte type, const byte *data, int16_t len, byte *out) {
    int16_t pos = 0, dpos = 0;

    out[pos++] = target;
    out[pos++] = type;

    while (dpos < len) {
        out[pos++] = data[dpos++];
    }

    return pos;
}

tiny_serial_handle_t port_handle;
Tiny::ProtoLight tx_protocol;
Tiny::ProtoLight rx_protocol;

void CmdVelCb(const sandbox_msgs::ChassisCommandConstPtr &msg) {
    steering_rad_ = std::max(std::min(-msg->steering, max_phi_plan_), min_phi_plan_);
    double steering = steering_rad_ / M_PI * 180;

    double throttle = std::max(std::min(msg->throttle * 100, 25.0), -25.0);
    TwistMessage twist = {
        int16_t(throttle),
        int16_t(steering),
    };

    byte buf[100] = {0};
    int16_t size = GenerateMessage((unsigned char)msg->target, MSG_TYPE_TWIST, (byte *)&twist,
                                    sizeof(TwistMessage), buf);
    tx_protocol.write(reinterpret_cast<char *>(buf), size);
}

int serial_send_fd(void *p, const void *buf, int len) {
    return tiny_serial_send(port_handle, buf, len);
}

int serial_receive_fd(void *p, void *buf, int len) {
    return tiny_serial_read(port_handle, buf, len);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "chassis_node");
    ros::NodeHandle nh;

    port_handle = tiny_serial_open("/dev/ttyUSB0", 57600);
    if (port_handle == TINY_SERIAL_INVALID) {
        std::cerr << "Error opening serial port" << std::endl;
        return 1;
    }
    tx_protocol.enableCheckSum();
    rx_protocol.enableCheckSum();
    tx_protocol.begin(serial_send_fd, serial_receive_fd);
    rx_protocol.begin(serial_send_fd, serial_receive_fd);
    ros::Publisher vel_publisher =
        nh.advertise<std_msgs::Float64>("/velocity", 1, false);
    velocity_publisher_ = &vel_publisher;
    ros::Subscriber cmd_subscriber =
        nh.subscribe("/chassis", 10, CmdVelCb);
    std::atomic<bool> terminate{false};
    std::thread rx_thread([&terminate] {
        Tiny::Packet<12> packet;
        while (!terminate.load()) {
            int len = rx_protocol.read(packet);
            if (len > 1) {
                byte* data = reinterpret_cast<byte*>(packet.data());
                HandleMessage(data[0], data + 1, len - 1);
            }
        }
    });
    ros::spin();
    terminate.store(true);
    rx_thread.join();
    rx_protocol.end();
    tx_protocol.end();

    tiny_serial_close(port_handle);
    return 0;
}

