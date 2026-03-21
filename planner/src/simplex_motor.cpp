#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <math.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define track_width 1.005
#define wheel_diameter 0.30
#define max_wheel_RPM 100

class MotorController : public rclcpp::Node
{
public:
    MotorController() : Node("motor_controller")
    {
        max_linear_velocity = ((M_PI * wheel_diameter) * max_wheel_RPM) / 60;
        pwm_start = 0;
        direction = 0;

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1, std::bind(&MotorController::vel_callback, this, std::placeholders::_1));

        Ena = 13;
        Enb = 12;
        Dira = 19;
        Dirb = 6;

        init_socket();
    }

    ~MotorController()
    {
        close(sock);
    }

    void updateMotor()
    {
        if (pwm_start == 1)
        {
            std::string pwm_data = formatPWMData();

            sendPWMData(pwm_data);
        }
        if (pwm_start == 0)
        {
            std::string pwm_data = formatPWMDataaa();

            sendPWMData(pwm_data);
        }
    }

private:
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        pwm_start = 1;
        linear_vel = msg->linear.x * 1.2;

        angular_vel = -msg->angular.z * 1.2;

        if (linear_vel == 0 && angular_vel == 0)
        {
            pwm_start = 0;
        }

        if (linear_vel > max_linear_velocity)
        {
            linear_vel = max_linear_velocity;
        }
        else if (linear_vel < -max_linear_velocity)
        {
            linear_vel = -max_linear_velocity;
        }

        findMaxAngular();

        if (angular_vel > maxAngularVel)
        {
            angular_vel = maxAngularVel;
        }
        else if (angular_vel < -maxAngularVel)
        {
            angular_vel = -maxAngularVel;
        }

        wheelVelocity();
    }

    void findMaxAngular()
    {
        maxAngularVel = (max_linear_velocity - linear_vel) * 2 / track_width;
    }

    void wheelVelocity()
    {
        right_vel = fabs(linear_vel + angular_vel * track_width / 2);
        left_vel = fabs(linear_vel - angular_vel * track_width / 2);
        setRPM();
        setDirection();
    }

    void setDirection()
    {
        if ((linear_vel + angular_vel * track_width / 2) > 0 && (linear_vel - angular_vel * track_width / 2) > 0)
        {
            direction = 1;
        }
        else if ((linear_vel + angular_vel * track_width / 2) < 0 && (linear_vel - angular_vel * track_width / 2) < 0)
        {
            direction = 2;
        }
        else if ((linear_vel + angular_vel * track_width / 2) < 0 && (linear_vel - angular_vel * track_width / 2) > 0)
        {
            direction = 3;
        }
        else if ((linear_vel + angular_vel * track_width / 2) > 0 && (linear_vel - angular_vel * track_width / 2) < 0)
        {
            direction = 4;
        }
    }

    void setRPM()
    {
        right_rpm = right_vel * 60 / (wheel_diameter * M_PI);
        left_rpm = left_vel * 60 / (wheel_diameter * M_PI);

        right_dutyCyclePercentage = right_rpm / max_wheel_RPM * 100;
        left_dutyCyclePercentage = left_rpm / max_wheel_RPM * 100;

        if (right_dutyCyclePercentage > 100)
            right_dutyCyclePercentage = 99;
        if (left_dutyCyclePercentage > 100)
            left_dutyCyclePercentage = 99;
    }

    void init_socket()
    {
        // Create a UDP socket (SOCK_DGRAM)
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }

        // Set up the server address (target IP and port)
        server.sin_family = AF_INET;
        server.sin_port = htons(5005);
        inet_pton(AF_INET, "10.0.0.7", &server.sin_addr);

        RCLCPP_INFO(this->get_logger(), "Socket created for UDP, ready to send data");
    }

    std::string formatPWMData()
    {
        // if (direction == 3) right_dutyCyclePercentage *= -1;
        // if (direction == 4) left_dutyCyclePercentage *= -1;

        if (direction == 3)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ("L:-" + std::to_string(left_dutyCyclePercentage) + " R:" + std::to_string(right_dutyCyclePercentage)).c_str());
            return "L-" + std::to_string(int(round(left_dutyCyclePercentage))) + "R" + std::to_string(int(round(right_dutyCyclePercentage))) + "E";
        }
        else if (direction == 4)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ("L:" + std::to_string(left_dutyCyclePercentage) + " R:-" + std::to_string(right_dutyCyclePercentage)).c_str());
            return "L" + std::to_string(int(round(left_dutyCyclePercentage))) + "R-" + std::to_string(int(round(right_dutyCyclePercentage))) + "E";
        }

        else if (direction == 1)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ("L:" + std::to_string(left_dutyCyclePercentage) + " R:" + std::to_string(right_dutyCyclePercentage)).c_str());
            return "L" + std::to_string(int(round(left_dutyCyclePercentage))) + "R" + std::to_string(int(round(right_dutyCyclePercentage))) + "E";
        }
        else if (direction == 2)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ("L:-" + std::to_string(left_dutyCyclePercentage) + " R:-" + std::to_string(right_dutyCyclePercentage)).c_str());
            return "L-" + std::to_string(int(round(left_dutyCyclePercentage))) + "R-" + std::to_string(int(round(right_dutyCyclePercentage))) + "E";
        }

        // return "L90R90";
    }

    std::string formatPWMDataaa()
    {
        // if (direction == 3) right_dutyCyclePercentage *= -1;
        // if (direction == 4) left_dutyCyclePercentage *= -1;
        RCLCPP_INFO(this->get_logger(), "%s", ("yoooo"));
        return "L0R0E";
    }

    void sendPWMData(const std::string &data)
    {
        // Send data using sendto (UDP is connectionless)
        ssize_t sent_bytes = sendto(sock, data.c_str(), data.size(), 0,
                                    (struct sockaddr *)&server, sizeof(server));
        if (sent_bytes < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data over UDP");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Sent UDP data: %s", data.c_str());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    int pwm_start, Ena, Enb, Dira, Dirb, sock;
    float right_dutyCyclePercentage, left_dutyCyclePercentage;
    int direction;
    float linear_vel, angular_vel, right_vel, left_vel, right_rpm, left_rpm, maxAngularVel, max_linear_velocity;
    struct sockaddr_in server;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::Rate rate(60);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->updateMotor();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}