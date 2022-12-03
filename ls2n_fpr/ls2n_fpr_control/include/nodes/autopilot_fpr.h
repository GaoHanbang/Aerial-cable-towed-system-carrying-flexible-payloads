#ifndef AUTOPILOT_FPR
#define AUTOPILOT_FPR

#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"
#include "controller_fpr.h"
#include "observer_fpr.h"
#include "param_fpr.h"
#include "state_fpr.h"
#include "csv_logger.h"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ls2n_interfaces/msg/attitude_thrust_set_point.hpp"
#include "ls2n_interfaces/msg/drone_status.hpp"
#include "ls2n_interfaces/srv/drone_request.hpp"
#include "ls2n_fpr_interfaces/msg/fpr_trajectory.hpp"
#include "ls2n_fpr_interfaces/msg/fpr_wrench.hpp"
#include "ls2n_interfaces/msg/keep_alive.hpp"

using TransformStamped = geometry_msgs::msg::TransformStamped; 

class RosFprAutopilot : public rclcpp::Node
{
public:
    RosFprAutopilot();
    ~RosFprAutopilot();

private:
    bool getParams();
    void controlLoop();
    void transferKeepAlive(const ls2n_interfaces::msg::KeepAlive::SharedPtr& msg);
    void spinMotors(const std::shared_ptr<std_srvs::srv::Empty::Request>&,
                    const std::shared_ptr<std_srvs::srv::Empty::Response>&);
    void startExperiment(const std::shared_ptr<std_srvs::srv::Empty::Request>&,
                         const std::shared_ptr<std_srvs::srv::Empty::Response>&);
    void stopExperiment(const std::shared_ptr<std_srvs::srv::Empty::Request>&,
                         const std::shared_ptr<std_srvs::srv::Empty::Response>&);
    void startFlying();
    void updateDronePose(const nav_msgs::msg::Odometry::SharedPtr& msg, const int& index);
    void updateDroneStatus(const ls2n_interfaces::msg::DroneStatus::SharedPtr& msg, const int& index);
    void updatePlatformPose(const nav_msgs::msg::Odometry::SharedPtr& msg);
    void updateInteractionPose(const geometry_msgs::msg::Pose::SharedPtr& msg);
    void updateTrajectory(const ls2n_fpr_interfaces::msg::FprTrajectory::SharedPtr& msg);
    void checkDroneStatus();
    void broadcastFrames();
    void updateDesiredForceData(const std_msgs::msg::Float32::SharedPtr& msg);
    void publishExternalWrench();
    void logFlightData();

    rclcpp::TimerBase::SharedPtr timer;

    FPR::ParamFPR paramFpr;
    FPR::TrajFPR trajFpr;
    FPR::ControllerFPR::ControllerType controllerType;
    FPR::ControllerFPR* controllerFpr;
    FPR::WrenchObserverFPR* wrenchObserverFpr;
    FPR::ModelFPR* modelFpr;
    FPR::PoseFPR* poseFpr;
    StateVecType initPoseFpr;
    FPR::VelocityFPR* velocityFpr;
    FPR::DroneState poseDrone[NUM_LEG];
    std::vector<FPR::ControllerFPR::DroneTarget> droneTarget;

    double takeOffTime; // takeoff timestamp
    bool interactionSafe; // safety check for interaction experiment 

    // Boolean for starting interaction experiment
    bool interaction_exp;
    bool interaction_started;

    // Interaction pose
    struct InteractionPose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

    } interactionPose;

    // Actuation wrench acting on the FPR states
    DofVecType actuationWrench;

    // External wrench observation state
    bool enableWrenchObserver;
    DofVecType externalWrench; // total external wrench of FPR estimated

    int8_t status;
    std::array<int8_t, NUM_LEG> droneStatus;

    int droneIndexes[NUM_LEG];

    csvLogger* logger;
    std::string log_path;

    // TransformStamped msgs for all frames
    struct FprFrames
    {
        TransformStamped platform_frame;
        TransformStamped leg_frames[NUM_LEG];
        TransformStamped drone_frames[NUM_LEG];
        TransformStamped drone_des_frames[NUM_LEG];
        static void set_frame(TransformStamped& frame, const Eigen::Vector3d& p, const Eigen::Quaterniond& q, std::string frame_name)
        {
            frame.header.frame_id = "world";
            frame.child_frame_id = frame_name;
            frame.transform.translation = tf2::toMsg2(p);
            frame.transform.rotation = tf2::toMsg(q);
        };
    } framesFpr;

    // Bool value if broadcast frames to tf2
    bool broadcast_frames;
    // TransformBroadcaster for tf2 frames
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // Commuunication with trajectory publisher
    rclcpp::Subscription<ls2n_fpr_interfaces::msg::FprTrajectory>::SharedPtr trajSubscription; 

    // Subscription to external force measurements and desired state
    double desiredForce;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desiredForceSubscription;

    // Publisher for external wrench observer
    rclcpp::Publisher<ls2n_fpr_interfaces::msg::FprWrench>::SharedPtr externalWrenchPublisher;
    // Subscription to interaction pose
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr interactionPoseSubscription;
    
    // Communication with joystick
    rclcpp::Subscription<ls2n_interfaces::msg::KeepAlive>::SharedPtr keepAliveSubscription;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr spinMotorsService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startExperimentService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stopExperimentService;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startInteractionSubscription;

    // Communication with drone bridges
    rclcpp::Subscription<ls2n_interfaces::msg::DroneStatus>::SharedPtr droneStatusSubscription[NUM_LEG];
    rclcpp::Publisher<ls2n_interfaces::msg::KeepAlive>::SharedPtr keepAliveDronePublisher[NUM_LEG];
    rclcpp::Client<ls2n_interfaces::srv::DroneRequest>::SharedPtr requestClient[NUM_LEG];
    rclcpp::Publisher<ls2n_interfaces::msg::AttitudeThrustSetPoint>::SharedPtr attitudeThrustPublishDrone[NUM_LEG];

    // Communication with MOCAP
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomPlatformSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomDroneSubscription[NUM_LEG];
};

#endif