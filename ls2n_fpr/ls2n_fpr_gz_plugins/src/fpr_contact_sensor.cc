#include "fpr_contact_sensor.h"

#include <gazebo/plugins/ContactPlugin.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/gazebo_msgs.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>
#include <string>

namespace gazebo
{

class FprContactSensorPrivate
{
public:
    /// Callback to be called when sensor updates
    void OnUpdate();

    /// A pointer to the GazeboROS node
    gazebo_ros::Node::SharedPtr ros_node_{nullptr};

    /// Contact mesage publisher.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_{nullptr};

    /// Pointer to sensor
    sensors::ContactSensorPtr parent_sensor_;

    /// Connects to pre-render events.
    event::ConnectionPtr update_connection_;
};

FprContactSensor::FprContactSensor() : impl_(std::make_unique<FprContactSensorPrivate>())
{
}

FprContactSensor::~FprContactSensor()
{
    impl_->ros_node_.reset();
}

void FprContactSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    // Initialize ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Get the parent sensor
    impl_->parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(_sensor);
    
    // Make sure the parent sensor is valid
    if (!impl_->parent_sensor_) {
        RCLCPP_ERROR(
            impl_->ros_node_->get_logger(),
            "Contact sensor parent is not of type ContactSensor. Aborting");
        impl_->ros_node_.reset();
        return;
    }

    auto qos_sensor_fpr = rclcpp::SensorDataQoS();
    qos_sensor_fpr.keep_last(1);
    
    // Create Fpr contact sensor state publisher
    impl_->pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
        "/CommandCenter/fpr_force_sensor", qos_sensor_fpr);

    RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Publishing Fpr Contact Sensor States to [%s]",
        impl_->pub_->get_topic_name());

    impl_->update_connection_ = impl_->parent_sensor_->ConnectUpdated(
        std::bind(&FprContactSensorPrivate::OnUpdate, impl_.get()));

    impl_->parent_sensor_->SetActive(true);
}

void FprContactSensorPrivate::OnUpdate()
{
    msgs::Contacts contacts_msg;
    contacts_msg = parent_sensor_->Contacts();

    auto contacts = gazebo_ros::Convert<gazebo_msgs::msg::ContactsState>(contacts_msg);

    double contact_force = 0.0;
    for (auto contact_state: contacts.states)
    {
        for (auto wrench: contact_state.wrenches)
        {
            double fx = wrench.force.x; // need to change depending on the tool configuration (fx, or -fz)
            if (fx != 0.0)
                contact_force = fx;
        }
    }
    std_msgs::msg::Float32 sensor_msg;
    sensor_msg.data = contact_force;
    pub_->publish(sensor_msg);
}

GZ_REGISTER_SENSOR_PLUGIN(FprContactSensor)
}