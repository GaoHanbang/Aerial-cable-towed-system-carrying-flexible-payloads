#include "autopilot_fpr.h"
#include <utility>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "rcl_interfaces/srv/get_parameters.hpp"
#include <ctime>
#include <sstream>
#include <iomanip>
#include <boost/filesystem.hpp>

using namespace std::chrono_literals;
using DroneStatus = ls2n_interfaces::msg::DroneStatus;
using ControllerType = FPR::ControllerFPR::ControllerType;
using JacobianType = FPR::ModelFPR::JacobianType;

RosFprAutopilot::RosFprAutopilot():Node("autopilot_fpr",
                                        rclcpp::NodeOptions()
                                                .automatically_declare_parameters_from_overrides(true)),
                                   status(DroneStatus::IDLE) {
    if (!getParams())
    {
        RCLCPP_ERROR(get_logger(), "Parameter file or some parameters in the file are missing");
        exit(1);
    }
    paramFpr.computeParam();
    // Strengthen the qos sensor profile (only last kept, queue of 1)
    auto qos_sensor_fpr = rclcpp::SensorDataQoS();
    qos_sensor_fpr.keep_last(1);

    // Initialize the model and the controller
    modelFpr = new FPR::ModelFPR(paramFpr);
    poseFpr = new FPR::PoseFPR(paramFpr);
    velocityFpr = new FPR::VelocityFPR(*modelFpr);
    controllerFpr = new FPR::ControllerFPR(*modelFpr, controllerType);

    // Initialize external wrench observer
    DofVecType observerGains; observerGains.setOnes();
    observerGains.segment(0,3) *= paramFpr.observer_gain.k_p;
    observerGains.segment(3,3) *= paramFpr.observer_gain.k_o;
    observerGains.segment(6,NUM_LEG) *= paramFpr.observer_gain.k_l;
    wrenchObserverFpr = new FPR::WrenchObserverFPR(*modelFpr, observerGains);
    externalWrench.setZero();

    // Create trajectory subscription
    trajSubscription = create_subscription<ls2n_fpr_interfaces::msg::FprTrajectory>("/CommandCenter/fpr_trajectory",
                                                                                    qos_sensor_fpr,
                                                                                    [this](ls2n_fpr_interfaces::msg::FprTrajectory::SharedPtr traj){
                                                                                        updateTrajectory(traj);}
                                                                                    );

    // Create one keep alive subscription and publishers
    keepAliveSubscription = create_subscription<ls2n_interfaces::msg::KeepAlive>("KeepAlive",
                                                                                 qos_sensor_fpr,
                                                                                 [this](ls2n_interfaces::msg::KeepAlive::SharedPtr PH1) {
                                                                                 transferKeepAlive(PH1); } );
    for (int i=0; i<NUM_LEG; i++)
        keepAliveDronePublisher[i] = create_publisher<ls2n_interfaces::msg::KeepAlive>("/Drone"+std::to_string(droneIndexes[i])+"/KeepAlive", qos_sensor_fpr);

    // Creat subscriptions for drones status
    for (int i=0; i<NUM_LEG; i++)
        droneStatusSubscription[i] = create_subscription<ls2n_interfaces::msg::DroneStatus>("/Drone"+std::to_string(droneIndexes[i])+"/Status",
                                                                                            qos_sensor_fpr,
                                                                                            [this, i](ls2n_interfaces::msg::DroneStatus::SharedPtr PH1){
                                                                                                updateDroneStatus(PH1, i);}
                                                                                            );

    // Create one service to spin Motors
    spinMotorsService = create_service<std_srvs::srv::Empty>("SpinMotors",
                                                            [this](std::shared_ptr<std_srvs::srv::Empty::Request> r1,
                                                                    std::shared_ptr<std_srvs::srv::Empty::Response> r2)
                                                            { spinMotors(r1, r2); } );

    // Create one service to start experiment
    startExperimentService = create_service<std_srvs::srv::Empty>("StartExperiment",
                                                             [this](std::shared_ptr<std_srvs::srv::Empty::Request> r1,
                                                                    std::shared_ptr<std_srvs::srv::Empty::Response> r2)
                                                             { startExperiment(r1, r2); } );
    // Create service to stop experiment
    stopExperimentService = create_service<std_srvs::srv::Empty>("StopExperiment",
                                                             [this](std::shared_ptr<std_srvs::srv::Empty::Request> r1,
                                                                    std::shared_ptr<std_srvs::srv::Empty::Response> r2)
                                                             { stopExperiment(r1, r2); } );

    // Create subscriptions for robot state
    odomPlatformSubscription = create_subscription<nav_msgs::msg::Odometry>("/FPR/Mocap/odom",
                                                                          qos_sensor_fpr,
                                                                          [this](nav_msgs::msg::Odometry::SharedPtr PH1) {
                                                                              updatePlatformPose(PH1); });
    for (int i=0; i<NUM_LEG; i++)
        odomDroneSubscription[i] = create_subscription<nav_msgs::msg::Odometry>(
                "/Drone" + std::to_string(droneIndexes[i]) + "/EKF/odom",
                qos_sensor_fpr,
                [this, i](nav_msgs::msg::Odometry::SharedPtr PH1) {
                    updateDronePose(PH1, i);
                });

    // Create publisher for observed external wrench
    if (enableWrenchObserver)
    {
        externalWrenchPublisher = create_publisher<ls2n_fpr_interfaces::msg::FprWrench>(
            "/FPR/Observer/ExternalWrench",
            qos_sensor_fpr);
        RCLCPP_INFO(get_logger(), "External wrench observer created");
    }

    // Create subscription to desired interaction force (only if interaction experiment started)
    if (interaction_exp)
    {
        desiredForceSubscription = create_subscription<std_msgs::msg::Float32>(
           "/CommandCenter/fpr_desired_force",
           qos_sensor_fpr,
           [this](std_msgs::msg::Float32::SharedPtr msg) {
               updateDesiredForceData(msg);
           });
        interaction_started = false;
        startInteractionSubscription = create_subscription<std_msgs::msg::Bool>(
            "/CommandCenter/fpr_start_interaction",
            qos_sensor_fpr,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                interaction_started = msg->data;
            });
        interactionPoseSubscription = create_subscription<geometry_msgs::msg::Pose>(
            "/CommandCenter/fpr_interaction_pose",
            qos_sensor_fpr,
            [this](geometry_msgs::msg::Pose::SharedPtr msg) {
                updateInteractionPose(msg); 
            });
    }

    // Create client for requests to drones
    for (int i=0; i<NUM_LEG; i++)
        requestClient[i] = create_client<ls2n_interfaces::srv::DroneRequest>("/Drone"+std::to_string(droneIndexes[i])+"/Request");

    // Create publisher for drone command
    for (int i=0; i<NUM_LEG; i++)
        attitudeThrustPublishDrone[i] = create_publisher<ls2n_interfaces::msg::AttitudeThrustSetPoint>("/Drone"+std::to_string(droneIndexes[i])+"/AttitudeThrustSetPoint", qos_sensor_fpr);

    if(!get_parameter("enable_rviz", broadcast_frames))
        broadcast_frames = false;
    else
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create the control loop
    timer = create_wall_timer(20ms, [this] { controlLoop(); });

    std::string control_msg = "None";
    if (controllerType == ControllerType::PD_CTC)
        control_msg = "PD";
    else if (controllerType == ControllerType::PID_CTC)
        control_msg = "PID";
    else if (controllerType == ControllerType::IMPEDANCE)
    {
        control_msg = "Impedance";
        if (!enableWrenchObserver)
            RCLCPP_WARN(get_logger(), "External wrench observer disabled, but needed for Impedance Control");
    }   

    // Get current date and time
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    auto date = oss.str();
    // Initialize csv logger
    boost::filesystem::create_directories(log_path);
    logger = new csvLogger(log_path+"/fpr_log_"+date+".csv");
    std::vector<std::string> topics = {"Drone1.Odom", "Drone2.Odom", "Drone3.Odom",
        "Drone1.Setpoint", "Drone2.Setpoint", "Drone3.Setpoint", 
        "FPR.Pose", "FPR.Traj", "FPR.ExternalWrench", "FPR.DesiredForce"};
    logger->init(topics);

    RCLCPP_INFO(get_logger(), "Fpr Autopilot Initialized with %s Control", control_msg.c_str());
    if (interaction_exp)
        RCLCPP_INFO(get_logger(), "Starting Interaction Experiment");
}

RosFprAutopilot::~RosFprAutopilot()
{
    delete modelFpr;
    delete poseFpr;
    delete velocityFpr;
    delete controllerFpr;
    delete wrenchObserverFpr;
}

void RosFprAutopilot::controlLoop() {
    checkDroneStatus();

    // Updates current pose
    poseFpr->getLegFromDrones(poseDrone);
    StateVecType robotPose = poseFpr->getPose();
    controllerFpr->updateCurrentPose(robotPose);

    // Update current velocity
    velocityFpr->getVelFromDrones(poseDrone, robotPose);
    DofVecType robotVelocity = velocityFpr->getVelocity();
    controllerFpr->updateCurrentVelocity(robotVelocity);
    
    if (status == DroneStatus::FLYING)
    {
        if (enableWrenchObserver)
        {
            // Update external wrench observation
            externalWrench = wrenchObserverFpr->updateEstimate(actuationWrench, robotPose, robotVelocity, get_clock()->now().seconds());
            if (controllerType==ControllerType::IMPEDANCE)
                controllerFpr->updateExternalWrench(externalWrench);
            publishExternalWrench();
        }

        // Updates desired trajectory
        controllerFpr->updateTrajectory(trajFpr.pose, trajFpr.velocity, trajFpr.acceleration);

        // Computes the controller
        std::string err;
        controllerFpr->spinController(err);
        if (!err.empty())
            RCLCPP_ERROR(get_logger(), "%s", ("Controller error: "+ err).c_str());

        // Save the actuation wrench computed
        actuationWrench = controllerFpr->getActuationWrench();

        // Send command to the drones
        controllerFpr->getDroneTargets(droneTarget);
        for (int i = 0; i<3; i++)
        {
            auto msg = ls2n_interfaces::msg::AttitudeThrustSetPoint();
            geometry_msgs::msg::Quaternion attitude;
            attitude.set__w(droneTarget[i].attitude.w());
            attitude.set__x(droneTarget[i].attitude.x());
            attitude.set__y(droneTarget[i].attitude.y());
            attitude.set__z(droneTarget[i].attitude.z());
            msg.set__attitude(attitude);
            msg.set__thrust(droneTarget[i].thrust);
            attitudeThrustPublishDrone[i]->publish(msg);
        }
        logFlightData();
    }
    else if (status == DroneStatus::IDLE || status == DroneStatus::ARMED)
    {
        // update initial position
        initPoseFpr <<  poseFpr->platform_position, 
                        poseFpr->platform_orientation.vec(), 
                        poseFpr->platform_orientation.w(),
                        poseFpr->leg_angles;
    }
    else if (status == DroneStatus::EMERGENCY_STOP)
    {
        for (int i = 0; i<3; i++)
        {
            geometry_msgs::msg::Quaternion attitude;
            auto msg = ls2n_interfaces::msg::AttitudeThrustSetPoint();
            attitude.set__w(droneTarget[i].attitude.w());
            attitude.set__x(droneTarget[i].attitude.x());
            attitude.set__y(droneTarget[i].attitude.y());
            attitude.set__z(droneTarget[i].attitude.z());
            msg.set__attitude(attitude);
            msg.set__thrust(0.0);
            attitudeThrustPublishDrone[i]->publish(msg);
        }
    }

    if (broadcast_frames)
        broadcastFrames();
}

void RosFprAutopilot::transferKeepAlive(const ls2n_interfaces::msg::KeepAlive::SharedPtr& msg) {
    if (status == DroneStatus::EMERGENCY_STOP)
        msg->keep_alive = false;
    for (const auto &publisher: keepAliveDronePublisher)
        publisher->publish(*(msg.get()));
}

void RosFprAutopilot::spinMotors(const std::shared_ptr<std_srvs::srv::Empty::Request>&,
                                const std::shared_ptr<std_srvs::srv::Empty::Response>&) {
    if (status == DroneStatus::IDLE) {
        RCLCPP_INFO(get_logger(), "Spinning FPR motors");
        auto request = std::make_shared<ls2n_interfaces::srv::DroneRequest::Request>();
        request->request = ls2n_interfaces::srv::DroneRequest::Request::SPIN_MOTORS;
        for (const auto &client: requestClient)
            client->async_send_request(request);
        }
    else
        RCLCPP_ERROR(get_logger(), "Request refused: Not IDLE");
}

void RosFprAutopilot::startExperiment(const std::shared_ptr<std_srvs::srv::Empty::Request>&,
                                      const std::shared_ptr<std_srvs::srv::Empty::Response>&) {
    if (status == DroneStatus::ARMED){
        RCLCPP_INFO(get_logger(), "Starting experiment");
        auto request = std::make_shared<ls2n_interfaces::srv::DroneRequest::Request>();
        request->request = ls2n_interfaces::srv::DroneRequest::Request::ATTITUDE_THRUST_CONTROL;
        for(const auto &client:requestClient)
            client->async_send_request(request);
    }
    else
        RCLCPP_ERROR(get_logger(), "Request refused: Not ARMED");
}

void RosFprAutopilot::stopExperiment(const std::shared_ptr<std_srvs::srv::Empty::Request>&,
                                      const std::shared_ptr<std_srvs::srv::Empty::Response>&) {
    if (status == DroneStatus::FLYING){
        RCLCPP_INFO(get_logger(), "Stopping experiment");
        auto request = std::make_shared<ls2n_interfaces::srv::DroneRequest::Request>();
        request->request = ls2n_interfaces::srv::DroneRequest::Request::LAND;
        for(const auto &client:requestClient)
            client->async_send_request(request);
    }
    else
        RCLCPP_ERROR(get_logger(), "Request refused: Not FLYING");
}

void RosFprAutopilot::startFlying() {
    // Get the initial position and time and start controller
    std::cout << "Initial pose: " << std::endl;
    std::cout << "  Platform position: " << poseFpr->platform_position << std::endl;
    std::cout << "  Platform orientation: " << poseFpr->platform_orientation.vec() << " " << poseFpr->platform_orientation.w() << std::endl;
    std::cout << "  Leg angles: " << poseFpr->leg_angles << std::endl;

    if(interaction_exp)
    {
        // Print the interaction pose
        std::cout << "Interaction pose: " << std::endl;
        std::cout << "  position: " << interactionPose.position << std::endl;
        std::cout << "  orientation: " << interactionPose.orientation.coeffs() << std::endl;
    }
    status = DroneStatus::FLYING;
    takeOffTime = get_clock()->now().seconds();
    interactionSafe = true;
    RCLCPP_INFO(get_logger(), "Flying");
}

void RosFprAutopilot::updateDroneStatus(const ls2n_interfaces::msg::DroneStatus::SharedPtr& msg, const int& index) {
    droneStatus[index] = msg->status;
}

void RosFprAutopilot::checkDroneStatus() {
    auto allDrones = [this](int8_t s_test){return std::all_of(droneStatus.begin(), droneStatus.end(), [s_test](int8_t s){return s == s_test;});};
    auto anyDrone = [this](int8_t s_test){return std::any_of(droneStatus.begin(), droneStatus.end(), [s_test](int8_t s){return s == s_test;});};
    // Updates the status of the fpr node as function of the current status and the drones status
    if (status == DroneStatus::IDLE && allDrones(DroneStatus::ARMED)) {
        status = DroneStatus::ARMED;
        RCLCPP_INFO(get_logger(), "Drones are armed");
    }
    if (status == DroneStatus::ARMED && allDrones(DroneStatus::FLYING))
        startFlying();
    if (anyDrone(DroneStatus::EMERGENCY_STOP) && status != DroneStatus::EMERGENCY_STOP && status != DroneStatus::IDLE){
        status = DroneStatus::EMERGENCY_STOP;
        RCLCPP_INFO(get_logger(), "Emergency stop");
    }
    if (status == DroneStatus::FLYING){
        if (allDrones(DroneStatus::LANDING)){
            status = DroneStatus::LANDING;
            RCLCPP_INFO(get_logger(), "Landing");
        }
    }
}

void RosFprAutopilot::updateDronePose(const nav_msgs::msg::Odometry::SharedPtr& msg, const int& index) {
    poseDrone[index].timestamp = get_clock()->now().seconds();
    poseDrone[index].position.x() = msg->pose.pose.position.x;
    poseDrone[index].position.y() = msg->pose.pose.position.y;
    poseDrone[index].position.z() = msg->pose.pose.position.z;
    poseDrone[index].attitude.x() = msg->pose.pose.orientation.x;
    poseDrone[index].attitude.y() = msg->pose.pose.orientation.y;
    poseDrone[index].attitude.z() = msg->pose.pose.orientation.z;
    poseDrone[index].attitude.w() = msg->pose.pose.orientation.w;
    poseDrone[index].linear_velocity.x() = msg->twist.twist.linear.x;
    poseDrone[index].linear_velocity.y() = msg->twist.twist.linear.y;
    poseDrone[index].linear_velocity.z() = msg->twist.twist.linear.z;
}

void RosFprAutopilot::updatePlatformPose(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    poseFpr->platform_position.x() = msg->pose.pose.position.x;
    poseFpr->platform_position.y() = msg->pose.pose.position.y;
    poseFpr->platform_position.z() = msg->pose.pose.position.z;
    poseFpr->platform_orientation.x() = msg->pose.pose.orientation.x;
    poseFpr->platform_orientation.y() = msg->pose.pose.orientation.y;
    poseFpr->platform_orientation.z() = msg->pose.pose.orientation.z;
    poseFpr->platform_orientation.w() = msg->pose.pose.orientation.w;
}

void RosFprAutopilot::updateTrajectory(const ls2n_fpr_interfaces::msg::FprTrajectory::SharedPtr& traj)
{
    double dt = (get_clock()->now() - traj->header.stamp).seconds();
    if(abs(dt) > 0.5 ) // greater than 0.5 sec
        RCLCPP_WARN(get_logger(), "Received trajectory is out of time");
    trajFpr.pose(0) = traj->platform_pose.position.x;
    trajFpr.pose(1) = traj->platform_pose.position.y;
    trajFpr.pose(2) = traj->platform_pose.position.z;
    trajFpr.pose(3) = traj->platform_pose.orientation.x;
    trajFpr.pose(4) = traj->platform_pose.orientation.y;
    trajFpr.pose(5) = traj->platform_pose.orientation.z;
    trajFpr.pose(6) = traj->platform_pose.orientation.w;
    trajFpr.velocity(0) = traj->platform_twist.linear.x;
    trajFpr.velocity(1) = traj->platform_twist.linear.y;
    trajFpr.velocity(2) = traj->platform_twist.linear.z;
    trajFpr.velocity(3) = traj->platform_twist.angular.x;
    trajFpr.velocity(4) = traj->platform_twist.angular.y;
    trajFpr.velocity(5) = traj->platform_twist.angular.z;
    trajFpr.acceleration(0) = traj->platform_accel.linear.x;
    trajFpr.acceleration(1) = traj->platform_accel.linear.y;
    trajFpr.acceleration(2) = traj->platform_accel.linear.z;
    trajFpr.acceleration(3) = traj->platform_accel.angular.x;
    trajFpr.acceleration(4) = traj->platform_accel.angular.y;
    trajFpr.acceleration(5) = traj->platform_accel.angular.z;
    for(int i=0; i<NUM_LEG; i++)
    {
        trajFpr.pose(7+i) = traj->leg_angles.at(i).data;
        trajFpr.velocity(6+i) = traj->leg_angles_vel.at(i).data;
        trajFpr.acceleration(6+i) = traj->leg_angles_accel.at(i).data;
    }
    // adding interaction pose or initial platform position in the desired trajectory
    if (interaction_exp)
    {
        static Eigen::Vector3d inter_pose;
        inter_pose = interactionPose.position - interactionPose.orientation.toRotationMatrix()*Eigen::Vector3d(0,0,0.05);
        trajFpr.pose.segment(0,2) = inter_pose.segment(0,2);
        if (interaction_started && interactionSafe)
        {
            trajFpr.pose.segment(0,3) = inter_pose;
            Eigen::Quaterniond q_composed, q_traj;
            q_traj.coeffs() = trajFpr.pose.segment(3,4);
            q_composed = interactionPose.orientation*q_traj;
            trajFpr.pose.segment(3,4) = q_composed.coeffs();

            // For savety, limit the interaction experiment during 50s
            if (get_clock()->now().seconds() - takeOffTime > 50.0)
            {
                RCLCPP_WARN(get_logger(), "Flight lasts over 50s, switch off interaction task automatically for safety");
                interactionSafe = false;
            }
        }
    }
    else
        trajFpr.pose.segment(0,2) += initPoseFpr.segment(0,2);
}

void RosFprAutopilot::updateInteractionPose(const geometry_msgs::msg::Pose::SharedPtr& msg)
{
    if (status == DroneStatus::FLYING) // only update if the FPR is not flying yet
        return;
    interactionPose.position[0] = msg->position.x;
    interactionPose.position[1] = msg->position.y;
    interactionPose.position[2] = msg->position.z;
    interactionPose.orientation.w() = msg->orientation.w;
    interactionPose.orientation.x() = msg->orientation.x;
    interactionPose.orientation.y() = msg->orientation.y;
    interactionPose.orientation.z() = msg->orientation.z;
}

void RosFprAutopilot::updateDesiredForceData(const std_msgs::msg::Float32::SharedPtr& msg)
{
    desiredForce = double(msg->data);
    DofVecType desiredForceVec; 
    desiredForceVec.setZero();   
    Eigen::Vector3d forceVec(0,0,desiredForce); // desired force is towards positive z axis of the tool
    Eigen::Matrix3d Rp = poseFpr->platform_orientation.toRotationMatrix();
    desiredForceVec.segment(0,3) = Rp*forceVec;

    controllerFpr->updateDesiredWrench(desiredForceVec);
}

void RosFprAutopilot::broadcastFrames() {
    std::vector<geometry_msgs::msg::TransformStamped> tf_msg;
    // Platform frame
    FprFrames::set_frame(framesFpr.platform_frame, 
            poseFpr->platform_position, 
            poseFpr->platform_orientation, 
            "Platform");
    framesFpr.platform_frame.header.stamp = get_clock()->now();
    tf_msg.push_back(framesFpr.platform_frame);
    // Legs and drones frames
    Eigen::Matrix3d Rp = poseFpr->platform_orientation.toRotationMatrix();
    for(int i=0; i < NUM_LEG; i++)
    {
        Eigen::Vector3d p_leg = poseFpr->platform_position + Rp*paramFpr.pos_ri[i]; // position of leg i's frame expressed in frame 0
        Eigen::Matrix3d Rleg = buildUnitRotMat(paramFpr.alpha[i],'z')*buildUnitRotMat(-M_PI/2,'x')*buildUnitRotMat(poseFpr->leg_angles(i),'z');
        Eigen::Quaterniond q_leg(Rp*Rleg);  
        // set leg frames     
        FprFrames::set_frame(framesFpr.leg_frames[i], 
            p_leg, q_leg, 
            "Leg"+std::to_string(i+1));
        // set drone frames
        FprFrames::set_frame(framesFpr.drone_frames[i], 
            poseDrone[i].position, poseDrone[i].attitude, 
            "Drone"+std::to_string(droneIndexes[i]));
        // set desired drone frames
        if (droneTarget.empty() || droneTarget.size() != NUM_LEG)
        {
            // set the current frames if desired frames are not computed yet
            framesFpr.drone_des_frames[i] = framesFpr.drone_frames[i];
        }
        else
        {
            FprFrames::set_frame(framesFpr.drone_des_frames[i],
                poseDrone[i].position, droneTarget.at(i).attitude,
                "Des_Drone"+std::to_string(droneIndexes[i]));
        }   
        framesFpr.leg_frames[i].header.stamp = get_clock()->now();
        framesFpr.drone_frames[i].header.stamp = get_clock()->now();
        framesFpr.drone_des_frames[i].header.stamp = get_clock()->now();
        tf_msg.push_back(framesFpr.leg_frames[i]);
        tf_msg.push_back(framesFpr.drone_frames[i]);
        tf_msg.push_back(framesFpr.drone_des_frames[i]);
    }
    // Broadcast frames
    tfBroadcaster->sendTransform(tf_msg);
}

void RosFprAutopilot::publishExternalWrench()
{
    auto msg = ls2n_fpr_interfaces::msg::FprWrench();
    msg.platform_wrench.force.x = externalWrench[0];
    msg.platform_wrench.force.y = externalWrench[1];
    msg.platform_wrench.force.z = externalWrench[2];
    msg.platform_wrench.torque.x = externalWrench[3];
    msg.platform_wrench.torque.y = externalWrench[4];
    msg.platform_wrench.torque.z = externalWrench[5];
    for (int i=0; i<NUM_LEG; i++)
        msg.leg_moments[i].data = externalWrench[6+i];
   
    externalWrenchPublisher->publish(msg);
}

void RosFprAutopilot::logFlightData()
{
    double curr_time = get_clock()->now().seconds();
    // logging timestamped drone poses
    for (auto & pose : poseDrone)
    {
        Eigen::Matrix<double,8,1> drone_odom_timestamped;
        drone_odom_timestamped << pose.timestamp, pose.position,
            pose.attitude.w(), pose.attitude.vec();
        logger->write(drone_odom_timestamped);
    }
    // logging timestamped drone setpoints
    for (int i=0; i<NUM_LEG; i++)
    {
        Eigen::Matrix<double,6,1> drone_setpoint_timestamped;
        drone_setpoint_timestamped << curr_time, droneTarget[i].thrust,
            droneTarget[i].attitude.w(), droneTarget[i].attitude.vec();
        logger->write(drone_setpoint_timestamped);
    }
    // logging FPR pose
    Eigen::Matrix<double,11,1> fpr_pose_timestamped;
    fpr_pose_timestamped << curr_time, poseFpr->getPose();
    logger->write(fpr_pose_timestamped);

    Eigen::Matrix<double,11,1> fpr_traj_timestamped;
    fpr_traj_timestamped << curr_time, trajFpr.pose;
    logger->write(fpr_traj_timestamped);

    // logging external wrench estimates
    Eigen::Matrix<double,10,1> external_wrench_timestamped;
    external_wrench_timestamped << curr_time, externalWrench;
    logger->write(external_wrench_timestamped);

    // logging desired force
    Eigen::Matrix<double,2,1> desired_force_timestamped;
    desired_force_timestamped << curr_time, desiredForce;
    logger->write(desired_force_timestamped);
}

bool RosFprAutopilot::getParams() 
{
    // Load control type param and basic params
    std::string ctrl_type;
    bool loaded = get_parameter("control_param.type", ctrl_type) &&
        get_parameter("basic_param.l_leg", paramFpr.l_leg) &&
        get_parameter("basic_param.r_p", paramFpr.r_p) &&
        get_parameter("basic_param.mass_p", paramFpr.mp) &&
        get_parameter("basic_param.mass_leg", paramFpr.mleg) &&
        get_parameter("basic_param.sx_leg", paramFpr.sleg.x()) &&
        get_parameter("basic_param.xx_p", paramFpr.Ip(0,0)) &&
        get_parameter("basic_param.yy_p", paramFpr.Ip(1,1)) &&
        get_parameter("basic_param.zz_p", paramFpr.Ip(2,2)) &&
        get_parameter("basic_param.xx_leg", paramFpr.Ileg(0,0)) &&
        get_parameter("basic_param.yy_leg", paramFpr.Ileg(1,1)) &&
        get_parameter("basic_param.zz_leg", paramFpr.Ileg(2,2));

    if(strcasecmp(ctrl_type.c_str(), "pd")==0)
        controllerType = ControllerType::PD_CTC;
    else if(strcasecmp(ctrl_type.c_str(), "pid")==0)
        controllerType = ControllerType::PID_CTC;
    else if(strcasecmp(ctrl_type.c_str(), "impedance")==0)
        controllerType = ControllerType::IMPEDANCE;
    else
    {
        RCLCPP_ERROR(get_logger(), "ControlType not defined, control_param.type parameter is invalid");
        return false;
    }

    // Load boolean for interaction experiment
    loaded = loaded && get_parameter("interaction_experiment", interaction_exp);

    // Load drone params
    loaded = loaded && get_parameter("drone_param.drone0.num", droneIndexes[0]) &&
        get_parameter("drone_param.drone1.num", droneIndexes[1]) &&
        get_parameter("drone_param.drone2.num", droneIndexes[2]) &&
        get_parameter("drone_param.drone0.angle", paramFpr.beta[0]) &&
        get_parameter("drone_param.drone1.angle", paramFpr.beta[1]) &&
        get_parameter("drone_param.drone2.angle", paramFpr.beta[2]);

    //  Call DroneParameters service to get the drone mass
    for (int i=0; i<NUM_LEG; i++)
    {
        auto droneParametersClient = create_client<rcl_interfaces::srv::GetParameters>(
            "/Drone"+std::to_string(droneIndexes[i])+"/drone_bridge/get_parameters");
        while (!droneParametersClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "/Drone%d/drone_bridge/get_parameters client interrupted while waiting for service to appear", droneIndexes[i]);
                return false;
            }
            RCLCPP_INFO(get_logger(), "Waiting for service /Drone%d/drone_bridge/get_parameters to appear...", droneIndexes[i]);
        }
        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        request->names = {"mass"};
        auto result_future = droneParametersClient->async_send_request(request);
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "/Drone%d/drone_bridge/get_parameters service call failed", droneIndexes[i]);
            return false;
        }
        paramFpr.m_drones[i] = result_future.get()->values[0].double_value;
    }
    
    // Load controller params
    if (controllerType == ControllerType::PD_CTC || controllerType == ControllerType::PID_CTC)
    {
        loaded = loaded && get_parameter("control_param.pid_gains.position.kp_xy", paramFpr.ctrl_gain.kp_p[0]) &&
            get_parameter("control_param.pid_gains.position.kp_xy", paramFpr.ctrl_gain.kp_p[1]) &&
            get_parameter("control_param.pid_gains.position.kp_z", paramFpr.ctrl_gain.kp_p[2]) &&
            get_parameter("control_param.pid_gains.position.kd_xy", paramFpr.ctrl_gain.kd_p[0]) &&
            get_parameter("control_param.pid_gains.position.kd_xy", paramFpr.ctrl_gain.kd_p[1]) &&
            get_parameter("control_param.pid_gains.position.kd_z", paramFpr.ctrl_gain.kd_p[2]) &&
            get_parameter("control_param.pid_gains.orientation.kp_xy", paramFpr.ctrl_gain.kp_o[0]) &&
            get_parameter("control_param.pid_gains.orientation.kp_xy", paramFpr.ctrl_gain.kp_o[1]) &&
            get_parameter("control_param.pid_gains.orientation.kp_z", paramFpr.ctrl_gain.kp_o[2]) &&
            get_parameter("control_param.pid_gains.orientation.kd_xy", paramFpr.ctrl_gain.kd_o[0]) &&
            get_parameter("control_param.pid_gains.orientation.kd_xy", paramFpr.ctrl_gain.kd_o[1]) &&
            get_parameter("control_param.pid_gains.orientation.kd_z", paramFpr.ctrl_gain.kd_o[2]) &&
            get_parameter("control_param.pid_gains.leg_angle.kp_l", paramFpr.ctrl_gain.kp_l) &&
            get_parameter("control_param.pid_gains.leg_angle.kd_l", paramFpr.ctrl_gain.kd_l) &&
            get_parameter("control_param.pid_gains.position.ki_xy", paramFpr.ctrl_gain.ki_p[0]) &&
            get_parameter("control_param.pid_gains.position.ki_xy", paramFpr.ctrl_gain.ki_p[1]) &&
            get_parameter("control_param.pid_gains.position.ki_z", paramFpr.ctrl_gain.ki_p[2]) &&
            get_parameter("control_param.pid_gains.orientation.ki_xy", paramFpr.ctrl_gain.ki_o[0]) &&
            get_parameter("control_param.pid_gains.orientation.ki_xy", paramFpr.ctrl_gain.ki_o[1]) &&
            get_parameter("control_param.pid_gains.orientation.ki_z", paramFpr.ctrl_gain.ki_o[2]) &&
            get_parameter("control_param.pid_gains.leg_angle.ki_l", paramFpr.ctrl_gain.ki_l);
    }
    if (controllerType == ControllerType::IMPEDANCE)
    {
        loaded = loaded && get_parameter("control_param.impedance_gains.mv_p", paramFpr.ctrl_gain.mv_p) &&
            get_parameter("control_param.impedance_gains.dv_p", paramFpr.ctrl_gain.dv_p) &&
            get_parameter("control_param.impedance_gains.kv_p", paramFpr.ctrl_gain.kv_p) &&
            get_parameter("control_param.impedance_gains.mv_o", paramFpr.ctrl_gain.mv_o) &&
            get_parameter("control_param.impedance_gains.dv_o", paramFpr.ctrl_gain.dv_o) &&
            get_parameter("control_param.impedance_gains.kv_o", paramFpr.ctrl_gain.kv_o) &&
            get_parameter("control_param.impedance_gains.mv_l", paramFpr.ctrl_gain.mv_l) &&
            get_parameter("control_param.impedance_gains.dv_l", paramFpr.ctrl_gain.dv_l) &&
            get_parameter("control_param.impedance_gains.kv_l", paramFpr.ctrl_gain.kv_l);
    }

    // Load observation params
    loaded = loaded && get_parameter("observation_param.external_wrench.enable", enableWrenchObserver);
    if (enableWrenchObserver)
    {
        loaded = loaded &&
                get_parameter("observation_param.external_wrench.gains.k_p", paramFpr.observer_gain.k_p) &&
                get_parameter("observation_param.external_wrench.gains.k_o", paramFpr.observer_gain.k_o) &&
                get_parameter("observation_param.external_wrench.gains.k_l", paramFpr.observer_gain.k_l);
    }

    // Get log path
    loaded = loaded && get_parameter("logs_folder", log_path);  // TODO: Create log folder if not existing (io stream error)

    return loaded;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosFprAutopilot>());
    rclcpp::shutdown();
    return 0;
}