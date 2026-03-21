#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

static constexpr double track_width = 1.005;
static constexpr double wheel_diameter = 0.30;
static constexpr double max_wheel_RPM = 100.0;

static constexpr const char* ESP_IP = "10.0.0.7";
static constexpr int ESP_PORT = 5005;
static constexpr int LISTEN_PORT = 5010;

class RelayNode : public rclcpp::Node
{
public:
    RelayNode() : Node("relay_node_udp_bridge")
    {
        RCLCPP_INFO(get_logger(), "Relay node started");

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&RelayNode::cmdVelCallback, this, std::placeholders::_1));
        arm_sub_ = create_subscription<std_msgs::msg::String>("/arm_vel", 10, std::bind(&RelayNode::armCallback, this, std::placeholders::_1));
        mode_cmd_sub_ = create_subscription<std_msgs::msg::Bool>("/autonomous_mode_cmd", 10, std::bind(&RelayNode::modeCmdCallback, this, std::placeholders::_1));
        mode_state_pub_ = create_publisher<std_msgs::msg::Bool>("/autonomous_mode_state", 10);
        mode_timer_ = create_wall_timer(200ms, std::bind(&RelayNode::publishModeState, this));
        toggle_srv_ = create_service<std_srvs::srv::Trigger>("/toggle_autonomous", std::bind(&RelayNode::toggleAutonomousService, this, std::placeholders::_1, std::placeholders::_2));

        running_.store(true);

        sender_thread_ = std::thread(&RelayNode::udpSenderThread, this);
        listener_thread_ = std::thread(&RelayNode::udpListenerThread, this);
    }

    ~RelayNode()
    {
        shutdown();
    }

    void shutdown()
    {
        if (!running_.load()) return;
        RCLCPP_WARN(get_logger(), "Shutting down relay");

        running_.store(false);

        if (udp_tx_sock_ >= 0) close(udp_tx_sock_);
        if (udp_rx_sock_ >= 0) close(udp_rx_sock_);

        if (sender_thread_.joinable()) sender_thread_.join();
        if (listener_thread_.joinable()) listener_thread_.join();
    }

private:
    int udp_tx_sock_ = -1;
    int udp_rx_sock_ = -1;

    int arm_p_ = 0, arm_q_ = 0, arm_y_ = 0, arm_g_ = 0;

    std::mutex mtx_;
    std::atomic<bool> running_{true};
    std::atomic<bool> autonomous_mode_{false};

    std::string last_manual_packet_ = "M0X0Y0P0Q0A0S0J0DE";
    std::string last_motor_packet_ = "L0R0E";

    int extract(const std::string& s, char k)
    {
        auto i = s.find(k);
        if (i == std::string::npos) return 0;
        return std::stoi(s.substr(i + 1));
    }

    void armCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!autonomous_mode_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        arm_p_ = extract(msg->data, 'P');
        arm_q_ = extract(msg->data, 'Q');
        arm_y_ = extract(msg->data, 'Y');
        arm_g_ = extract(msg->data, 'G');
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!autonomous_mode_) return;

        double lin = msg->linear.x * 1.2;
        double ang = -msg->angular.z * 1.2;

        double maxv = (M_PI * wheel_diameter * max_wheel_RPM) / 60.0;
        lin = std::clamp(lin, -maxv, maxv);

        double maxw = (maxv - std::fabs(lin)) * 2.0 / track_width;
        ang = std::clamp(ang, -maxw, maxw);

        double r = lin + ang * track_width * 0.5;
        double l = lin - ang * track_width * 0.5;

        int rp = std::min(99, static_cast<int>(std::fabs(r) * 60.0 / (wheel_diameter * M_PI) / max_wheel_RPM * 100.0));
        int lp = std::min(99, static_cast<int>(std::fabs(l) * 60.0 / (wheel_diameter * M_PI) / max_wheel_RPM * 100.0));

        std::string p = "L" + std::string(l < 0 ? "-" : "") + std::to_string(lp) +
                        "R" + std::string(r < 0 ? "-" : "") + std::to_string(rp) + "E";

        std::lock_guard<std::mutex> lock(mtx_);
        last_motor_packet_ = p;
    }

    void udpSenderThread()
    {
        udp_tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);

        sockaddr_in esp{};
        esp.sin_family = AF_INET;
        esp.sin_port = htons(ESP_PORT);
        inet_pton(AF_INET, ESP_IP, &esp.sin_addr);

        RCLCPP_INFO(get_logger(), "UDP sender started");

        while (running_)
        {
            std::string pkt;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                if (!autonomous_mode_) pkt = last_manual_packet_;
                else
                {
                    std::string m = last_motor_packet_;
                    m.pop_back();
                    pkt = m + "P" + std::to_string(arm_p_) +
                          "Q" + std::to_string(arm_q_) +
                          "Y" + std::to_string(arm_y_) +
                          "G" + std::to_string(arm_g_) + "E";
                }
            }

            sendto(udp_tx_sock_, pkt.c_str(), pkt.size(), 0, (sockaddr*)&esp, sizeof(esp));

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200, "[ESP TX] %s", pkt.c_str());

            std::this_thread::sleep_for(50ms);
        }
    }

    void udpListenerThread()
    {
        udp_rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);

        sockaddr_in serv{};
        serv.sin_family = AF_INET;
        serv.sin_addr.s_addr = INADDR_ANY;
        serv.sin_port = htons(LISTEN_PORT);
        bind(udp_rx_sock_, (sockaddr*)&serv, sizeof(serv));

        RCLCPP_INFO(get_logger(), "UDP listener started");

        char buf[1500];

        while (running_)
        {
            ssize_t n = recv(udp_rx_sock_, buf, sizeof(buf), 0);
            if (n > 0 && !autonomous_mode_)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                last_manual_packet_ = std::string(buf, n);
            }
        }
    }

    void toggleAutonomousService(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r)
    {
        autonomous_mode_ = !autonomous_mode_;
        RCLCPP_WARN(get_logger(), "Autonomous -> %s", autonomous_mode_ ? "ON" : "OFF");
        r->success = true;
    }

    void modeCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        autonomous_mode_ = msg->data;
        RCLCPP_WARN(get_logger(), "Autonomous command -> %s", msg->data ? "ON" : "OFF");
    }

    void publishModeState()
    {
        std_msgs::msg::Bool b;
        b.data = autonomous_mode_;
        mode_state_pub_->publish(b);
    }

    std::thread sender_thread_, listener_thread_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_state_pub_;
    rclcpp::TimerBase::SharedPtr mode_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_srv_;
};

std::shared_ptr<RelayNode> node_ptr;

void sigintHandler(int)
{
    if (node_ptr) node_ptr->shutdown();
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    node_ptr = std::make_shared<RelayNode>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
}
