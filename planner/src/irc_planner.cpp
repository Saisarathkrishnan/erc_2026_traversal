#include <algorithm>
#include <cmath>
#include <limits>
#include "stack/irc_planner.hpp"
//

// using elog=elog;

//
// Constructor

Stack::Stack()
    : Node("planner_node"),

      angleSet(false),

      obs_x(0.0),
      obs_y(0.0),
      best_x(0.0),
      obstacle_detected_(false),
      cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
      roverAngle(0.0f)
{
    Coordinates err = {0, 0};
    attGoalArr[0] = goal(err, err, err);
    attGoalArr[1] = goal(err, err, err);
    attGoalArr[2] = goal(err, err, err);
    attGoalArr[3] = goal(err, err, err);

    stat123[elog::init] = 1;
    stat123[elog::goalInit] = 0;
    stat123[elog::gpsInit] = 0;
    stat123[elog::pclInit] = 0;
    stat123[elog::imuInit] = 0;
    stat123[elog::gpsActive] = 0;
    stat123[elog::pclActive] = 0;
    stat123[elog::imuActive] = 0;
    // stat123[elog::coolAsh]=0;
    stat123[elog::manual] = 0; // -1 manual
                               // 1 auto
    stat123[elog::currentGoal] = -1;

    // Time
    this->set_parameter(rclcpp::Parameter("use_sim_time", false));

    // Topics & Parameters
    declare_parameter("imu_topic", "/imu_data");
    declare_parameter("gps_topic", "/fix");
    declare_parameter("point_cloud_topic", "/local_grid_obstacle");
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("state_topic", "/autonomous_mode_state");

                declare_parameter<double>("min_x", 0.5);
        declare_parameter<double>("max_x", 3.0);
        declare_parameter<double>("half_width", 0.4);
        declare_parameter<double>("side_thresh", 0.15);
        declare_parameter<double>("log_rate_hz", 10.0);

        min_x_ = get_parameter("min_x").as_double();
        max_x_ = get_parameter("max_x").as_double();
        half_w_ = get_parameter("half_width").as_double();
        side_thresh_ = get_parameter("side_thresh").as_double();
        log_rate_hz_ = get_parameter("log_rate_hz").as_double();



    const auto imu_topic = get_parameter("imu_topic").as_string();
    const auto gps_topic = get_parameter("gps_topic").as_string();
    const auto cloud_topic = get_parameter("point_cloud_topic").as_string();
    const auto cmd_vel = get_parameter("cmd_vel_topic").as_string();

    const auto state_topic = get_parameter("state_topic").as_string();

    // Publishers
    vel_pub = create_publisher<geometry_msgs::msg::Twist>(cmd_vel, 10);

    // Subscribers
    imu_sub_ = create_subscription<custom_msgs::msg::ImuData>(imu_topic, 10, std::bind(&Stack::imuCallback, this, std::placeholders::_1));
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic, 10, std::bind(&Stack::gpsCallback, this, std::placeholders::_1));
    // cone_sub_ = create_subscription<custom_msgs::msg::MarkerTag>(cone_topic, 10, std::bind(&Stack::coneCallback, this, std::placeholders::_1));
    pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, 10, std::bind(&Stack::pclCallback, this, std::placeholders::_1));
    auto_sub_ = create_subscription<std_msgs::msg::Bool>(state_topic, 10, std::bind(&Stack::stateCallback, this, std::placeholders::_1));
    // gui_sub_ = create_subscription<custom_msgs::msg::GuiCommand>("/gui/command", 10,std::bind(&Stack::guiCommandCallback, this, std::placeholders::_1));

    // Timers & Services
    stack_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&Stack::stackRun, this));
    toggle_client_ = create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

    // Random Equations
    // obs_avoid_linear = straightLineEquation(kMinObsThreshold, kStopVel, kMaxObsThreshold, kMaxLinearVel);
    // obs_avoid_angular = straightLineEquation(kRoverBreadth / 2.0, kStopVel, kMinYObsThreshold, kMaxAngularVel);
    // obj_follow_linear = straightLineEquation(kMaxXObsDistThreshold, kMaxLinearVel, kMinXObsDistThreshold, kStopVel);
    // obj_follow_angular = straightLineEquation(kMinYObjDistThreshold, kStopVel, kMaxYObjDistThreshold, kMaxAngularVel);
}

void Stack::setGpsGoals()
{
    Coordinates inp = {0, 0};
    std::cout << "Enter waypoint 1+>>>>" << std::endl;
    std::cout << "enter latitude: ";
    std::cin >> inp.latitude;
    std::cout << "enter longitude: ";
    std::cin >> inp.longitude;

    attGoalArr[0].setGoal(inp, currGNSS);

    inp = {0, 0};
    std::cout << "Enter waypoint 2+>>>>" << std::endl;
    std::cout << "enter latitude: ";
    std::cin >> inp.latitude;
    std::cout << "enter longitude: ";
    std::cin >> inp.longitude;

    attGoalArr[1].setGoal(inp, currGNSS);

    inp = {0, 0};
    std::cout << "Enter waypoint 3+>>>>" << std::endl;
    std::cout << "enter latitude: ";
    std::cin >> inp.latitude;
    std::cout << "enter longitude: ";
    std::cin >> inp.longitude;

    attGoalArr[2].setGoal(inp, currGNSS);

    inp = {0, 0};
    std::cout << "Enter waypoint 4+>>>>" << std::endl;
    std::cout << "enter latitude: ";
    std::cin >> inp.latitude;
    std::cout << "enter longitude: ";
    std::cin >> inp.longitude;

    attGoalArr[3].setGoal(inp, currGNSS);

    inp = {0, 0};

    stat123[elog::goalInit] = 1;
}

void Stack::statesClassifier()
{
    if (PrimeNavState == NavStatus::Manual)
    {

        PrimeNavState = NavStatus::Manual;
    }
    if (PrimeNavState == NavStatus::Wait)
    {

        PrimeNavState = NavStatus::Wait;
    }
    else
    {
        if (stat123[elog::manual] == -1)
        {
            PrimeNavState = Manual;
        }

        if (stat123[elog::manual] == 1)
        {

            if (stat123[elog::goalInit] == 0)
            {
                PrimeNavState = GoalNotYetProvided;
            }

            if (stat123[elog::goalInit] == 1)
            {
                PrimeNavState = GoalSeeking;
            }
        }
    }
}
// Planner Main control loop
void Stack::stackRun()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    statesClassifier();

    switch (PrimeNavState)
    {
    case Manual:
        publishVel(geometry_msgs::msg::Twist());
        break;

    case GoalNotYetProvided:
        setGpsGoals();
        break;

    case Wait:
        waiting();
        break;

    case GoalSeeking:
        goalManager();
        break;
    case GnssError:
        break;
    default:
        publishVel(geometry_msgs::msg::Twist());
        break;
    }
    std::cout << "end" << std::endl;
}

void Stack::waiting()
{
    int wasd = 0;
    std::cout << "COntimnue yes(1) or no(anything) ===?";
    std::cin >> wasd;
    if (wasd == 1)
    {
        PrimeNavState = GoalSeeking;
    }
    else if (wasd == 2)
    {
        disableAutonomous();
        PrimeNavState = Manual;
    }
}
void Stack::goalManager()
{
    // fix it
    if (stat123[elog::currentGoal] == -1)
    {
        stat123[elog::currentGoal] = 0;
    }

    if (attGoalArr[stat123[elog::currentGoal]].GoalAchieved())
    {
        stat123[elog::currentGoal] += 1;
        PrimeNavState = Wait;
        return;
    }
    if (stat123[elog::currentGoal] > 3)
    {
        stat123[elog::manual] = -1;
    }
    // attGoalArr[stat123[elog::currentGoal]].update(currGNSS);
    coordinateFollowing(attGoalArr[stat123[elog::currentGoal]].GoalGNSS());
}

void Stack::imuCallback(const custom_msgs::msg::ImuData::SharedPtr imu_msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    roverAngle = imu_msg->orientation.z;

    stat123[elog::imuInit] = 1;
    stat123[elog::imuActive] = 1;
}

void Stack::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix_)
{
    if (fix_->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) // STATUS_FIX means at least a 2D GPS fix
        return;

    std::lock_guard<std::mutex> lock(state_mutex_);

    currGNSS.latitude = fix_->latitude;
    currGNSS.longitude = fix_->longitude;
    std::cout << currGNSS.latitude << ":" << currGNSS.latitude << std::endl;
    stat123[elog::gpsInit] = 1;
    stat123[elog::gpsActive] = 1;
}

void Stack::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_); // To make sure only one thread is using the protected variables :/

    bool found = false;
    float best_x = std::numeric_limits<float>::max();

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*cloud_msg, "y");

    for (; it_x != it_x.end(); ++it_x, ++it_y)
    {
        const float px = *it_x;
        const float py = *it_y;

        if (px < min_x_ || px > max_x_) continue;
        if (std::abs(py) > half_w_) continue;

        if (px < best_x)
        {
            best_x = px;
            obs_x = px;
            obs_y = py;
            found = true;
        }
    }
    
    obstacle_detected_ = found;
    std::cout<<"local_grid_obstacle yes"<<obstacle_detected_<<std::endl;
    obstacleClassifier();
    stat123[elog::pclInit] = 1;
    stat123[elog::pclActive] = 1;
}

void Stack::stateCallback(const std_msgs::msg::Bool::SharedPtr state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    bool Autostate = state->data;
    if (stat123[elog::manual] == -1)
    {
        if (!Autostate)
        {
            return;
        }
    }
    else if (stat123[elog::manual] == 1)
    {
        if (Autostate)
        {
            return;
        }
    }
    // PrimeNavState = state->data;

    if (!Autostate)
    {
        stat123[elog::manual] = -1;
        PrimeNavState = Manual;
        publishVel(geometry_msgs::msg::Twist());
        RCLCPP_WARN(this->get_logger(), "manual mode");
        return;
    }
    else if (Autostate)
    {
        stat123[elog::manual] = 1;
        PrimeNavState = GoalNotYetProvided;
        RCLCPP_INFO(this->get_logger(),
                    "auto ");
    }
}

void Stack::coordinateFollowing(Coordinates dest1)
{
    angleSet = false;
    geometry_msgs::msg::Twist speed = geometry_msgs::msg::Twist();

    double dist = haversine(currGNSS, dest1);
    double destAngle = gpsBearing(currGNSS, dest1);

    RCLCPP_INFO(this->get_logger(),
                "[GPS] dest=%.7f, %.7f distance =%.2f \n[GPS]rangle    =%.2f \n[GPS] angle    =%.2f", dest1.latitude, dest1.longitude, dist, roverAngle, destAngle);

    if (dist <= DistanceThreshold)
    {
        attGoalArr[stat123[elog::currentGoal]].GoalReached();
        RCLCPP_INFO(this->get_logger(),
                    "[GPS] Goal reached | remaining_dist=%.2f m", dist);

        hardStop();
        return;
    }

 

    double angle=std::abs(roverAngle - destAngle);
    //std::cout<<"hi"<<(std::abs(roverAngle - destAngle) < Angle_Tolerance)<<std::endl;
    if (!(std::abs(angle) < Angle_Tolerance))
    {
        std::cout<<"yes";
        angleSet = false;
        double angleDiffed = destAngle - roverAngle;
        angleDiffed=std::fmod((destAngle - roverAngle + 540.0), 360.0) - 180.0;
        std::cout<<angleDiffed<<std::endl;
        if (angleDiffed > 0)
        {
            //std::cout<<"1"<<angleDiffed<<std::endl;
            float speedIs = std::abs(pow(angleDiffed, 3) / 1000);
            if (speedIs > maxAngularSpeed)
            {
                speedIs = maxAngularSpeed;
            }
            if (speedIs < 0.1)
            {
                speedIs = 0.3;
            }
            speed.angular.z = -speedIs;
        }
        else if (angleDiffed < 0)
        {
            //std::cout<<"2"<<angleDiffed<<std::endl;
            float speedIs = std::abs(pow(angleDiffed, 3) / 1000);
            if (speedIs > maxAngularSpeed)
            {
                speedIs = maxAngularSpeed;
            }
            if (speedIs < 0.1)
            {
                speedIs = 0.3;
            }
            speed.angular.z = speedIs;
        }
    }
    else
    {
        std::cout<<"no";
        angleSet = true;

        double angleDiffed = destAngle - roverAngle;
        angleDiffed=std::fmod((destAngle - roverAngle + 540.0), 360.0) - 180.0;
        std::cout<<angleDiffed<<std::endl;
        if (angleDiffed > 0)
        {
            std::cout<<"1"<<angleDiffed<<std::endl;
            float speedIs = std::abs(pow(angleDiffed, 3) / 1000);
            if (speedIs > maxAngularSpeed)
            {
                speedIs = maxAngularSpeed;
            }
            if (speedIs < 0.1)
            {
                speedIs = 0.3;
            }
            speed.angular.z = -speedIs;
        }
        else if (angleDiffed < 0)
        {
            std::cout<<"2"<<angleDiffed<<std::endl;
            float speedIs = std::abs(pow(angleDiffed, 3) / 1000);
            if (speedIs > maxAngularSpeed)
            {
                speedIs = maxAngularSpeed;
            }
            if (speedIs < 0.1)
            {
                speedIs = 0.3;
            }
            speed.angular.z = speedIs;
        }
    }
    std::cout<<"on___-"<<obstacle_detected_<<std::endl;
    if(obstacle_detected_){
        double distance_lmao=std::hypot(obs_x, obs_y);
        std::cout<<"yes obstacle";
        if(std::abs(obs_y)<1.4){
            if(obs_y<1&&obs_y>0){
                speed.angular.z = 1;
                std::cout<<"turn left";
            }else if(obs_y>-1&&obs_y<0){
                speed.angular.z=-1;
                std::cout<<"turn right";
            }
        }
        std::cout<<"__"<<distance_lmao;
        if(distance_lmao<1.25){
            speed.linear.x=0;
            std::cout<<"below thres"<<std::endl;
        }else{
            speed.linear.x=pow(distance_lmao, 3) / 100;
        }
    }else{  

    if (angleSet)
    {
        double speedx = pow(dist, 3) / 100;
        if (speedx < 0.3)
        {
            speedx = 0.4;
        }
        if (speedx > 2)
        {
            speedx = 2;
        }
        speed.linear.x = 0.4;
    }}
    publishVel(speed);
}

void Stack::obstacleAvoidance()
{
}

void Stack::publishVel(const geometry_msgs::msg::Twist &msg)
{
    geometry_msgs::msg::Twist cmd = msg;

    vel_pub->publish(cmd);
}

void Stack::hardStop()
{
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.linear.y = 0.0;
    stop.linear.z = 0.0;
    stop.angular.x = 0.0;
    stop.angular.y = 0.0;
    stop.angular.z = 0.0;

    for (int i = 0; i < 5; ++i)
    {
        vel_pub->publish(stop);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
}

void Stack::disableAutonomous()
{
    if (!toggle_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "[MODE] toggle_autonomous service not available");
        return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    toggle_client_->async_send_request(
        req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            auto res = future.get();
            if (res->success)
            {
                RCLCPP_WARN(this->get_logger(), "[MODE] Autonomous DISABLED via service");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[MODE] Failed to disable autonomy: %s", res->message.c_str());
            }
        });
}

void Stack::obstacleClassifier()
{


}

// Computes the distance (in meters) between two GPS coordinates
double Stack::haversine(Coordinates curr, Coordinates dest)
{
    double lat1 = curr.latitude * M_PI / 180.0;
    double lat2 = dest.latitude * M_PI / 180.0;
    double dLat = lat2 - lat1;
    double dLon = (dest.longitude - curr.longitude) * M_PI / 180.0;

    double h = sin(dLat * 0.5) * sin(dLat * 0.5) +
               cos(lat1) * cos(lat2) * sin(dLon * 0.5) * sin(dLon * 0.5);

    return 2.0 * 6371000.0 * asin(sqrt(h));
}

// Computes the initial bearing (in degrees) from the current GPS coordinate to the destination GPS coordinate,
// measured clockwise from geographic North and normalized to the range [0, 360).
double Stack::gpsBearing(Coordinates curr, Coordinates dest)
{
    double lat1 = curr.latitude * M_PI / 180.0;
    double lon1 = curr.longitude * M_PI / 180.0;
    double lat2 = dest.latitude * M_PI / 180.0;
    double lon2 = dest.longitude * M_PI / 180.0;

    double dLon = lon2 - lon1;

    double x = sin(dLon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) -
               sin(lat1) * cos(lat2) * cos(dLon);

    double angle = atan2(x, y) * 180.0 / M_PI;
    if (angle < 0)
        angle += 360.0;

    return angle;
}

// Converts a GPS bearing (0° = North, clockwise positive) into the rover’s
// internal heading convention by applying a +90° frame shift, sign inversion,
// and wrapping the result to the range [-180, 180].
