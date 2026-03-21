#ifndef IRC_PLANNER_H_
#define IRC_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "custom_msgs/msg/marker_tag.hpp"
#include "custom_msgs/msg/imu_data.hpp"
#include "custom_msgs/msg/gui_command.hpp"
#include "custom_msgs/msg/planner_status.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "attGoalManager.hpp"
#include "head.hpp"

class Stack : public rclcpp::Node
{
public:
    Stack();
    ~Stack(){
        std::cout<<"crawzy stack deconstructor?"<<std::endl;
    }
private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    // Subscribers
    rclcpp::Subscription<custom_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_sub_;

    // Services & Timers
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_client_;
    rclcpp::TimerBase::SharedPtr stack_timer_;

    NavStatus PrimeNavState;
    goal attGoalArr[4];
  
    Coordinates currGNSS;

    
    bool angleSet;    
    double obs_x;
    double obs_y;
    double roverAngle;
    
    const float maxLinearSpeed=2;
    const float maxAngularSpeed=1.5;
    const float maxObsLinearSpeed=1;
    const float maxObsAngularSpeed=2;
    const float DistanceThreshold=5;
    const float Angle_Tolerance=10;


    std::map<elog,int> stat123; //has all states
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    std::mutex state_mutex_;

    // Core Functions
    void stackRun();
    void statesClassifier();
    void setGpsGoals();

    // Callbacks
    void imuCallback(const custom_msgs::msg::ImuData::SharedPtr imu_msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix);
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void stateCallback(const std_msgs::msg::Bool::SharedPtr state);

    // State Functions
    void coordinateFollowing(Coordinates dest1);
    void obstacleAvoidance();
    void navigationModeSelect();

    // Helper Functions
    void publishVel(const geometry_msgs::msg::Twist &msg);
    void hardStop();
    void disableAutonomous();
    void obstacleClassifier();
    void goalManager();
    void waiting();

    // Math Functions

    double haversine(Coordinates curr, Coordinates dest);
    double gpsBearing(Coordinates curr, Coordinates dest);


};

#endif
