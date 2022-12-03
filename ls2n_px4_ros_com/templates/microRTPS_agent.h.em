//
// Created by Damien SIX on 15/06/2021.
//

@###############################################
@#
@# EmPy template for generating microRTPS_agent.h file
@#
@###############################################

#ifndef MICRORTPS_AGENT_H
#define MICRORTPS_AGENT_H

#include "rclcpp/rclcpp.hpp"

// Default values
#define DEVICE "/dev/ttyACM0"
#define SLEEP_US 1
#define BAUDRATE 460800
#define POLL_MS 1
#define WAIT_CNST 2
#define DEFAULT_RECV_PORT 2020
#define DEFAULT_SEND_PORT 2019
#define DEFAULT_IP "127.0.0.1"

#include "microRTPS_transport.h"
#include "px4_msgs/msg/timesync.hpp"
#include "microRTPS_timesync.h"
#include <queue>
#include "std_msgs/msg/u_int64.hpp"

@[for head in topics_head]@
@[if head !="timesync"]@
#include "px4_msgs/msg/@(head).hpp"
@[end if]@
@[end for]@

// SFINAE
template<typename T> struct hasTimestampSample{
private:
    template<typename U,
            typename = decltype(std::declval<U>().set__timestamp_sample(int64_t()))>
    static std::true_type detect(int);
    template<typename U>
    static std::false_type detect(...);
public:
    static constexpr bool value = decltype(detect<T>(0))::value;
};

template<typename T>
inline typename std::enable_if<!hasTimestampSample<T>::value, uint64_t>::type
getMsgTimestampSample_impl(const T*) { return 0; }

/** Msg metada Getters **/
template <class T>
inline uint64_t getMsgTimestamp(const T* msg) { return msg->timestamp; }

template<typename T>
inline typename std::enable_if<hasTimestampSample<T>::value, uint64_t>::type
getMsgTimestampSample_impl(const T* msg) { return msg->timestamp_sample; }

template <class T>
inline uint64_t getMsgTimestampSample(const T* msg) { return getMsgTimestampSample_impl(msg); }

template<typename T>
inline typename std::enable_if<!hasTimestampSample<T>::value, void>::type
setMsgTimestampSample_impl(T*, const uint64_t&) {}

/** Msg metadata Setters **/
template <class T>
inline void setMsgTimestamp(T* msg, const uint64_t& timestamp) { msg->timestamp = timestamp; }

template <class T>
inline typename std::enable_if<hasTimestampSample<T>::value, void>::type
setMsgTimestampSample_impl(T* msg, const uint64_t& timestamp_sample) { msg->timestamp_sample = timestamp_sample; }

template <class T>
inline void setMsgTimestampSample(T* msg, const uint64_t& timestamp_sample) { setMsgTimestampSample_impl(msg, timestamp_sample); }

class MicroRtpsAgent: public rclcpp::Node
{
public:
    MicroRtpsAgent();
    bool connected = false;
    bool running;
    void CloseConnection() {if (transport_node) transport_node->close();}

private:
    void Publish(uint8_t topic_ID, char data_buffer[], size_t len);
    void MeanDelayPublish();
    void Receive();
    struct Options {
        enum class eTransports
        {
            UART,
            UDP
        };
        eTransports transport = Options::eTransports::UART;
        char device[64] = DEVICE;
        int sleep_us = SLEEP_US;
        int64_t baudrate = BAUDRATE;
        int poll_ms = POLL_MS;
        uint16_t recv_port = DEFAULT_RECV_PORT;
        uint16_t send_port = DEFAULT_SEND_PORT;
        char ip[16] = DEFAULT_IP;
        bool sw_flow_control = false;
        bool hw_flow_control = false;
        bool verbose_debug = false;
        std::string ns = "";
    } options;

    Transport_node *transport_node = nullptr;
    uint32_t total_sent = 0, sent = 0;
    char data_buffer[BUFFER_SIZE] = {};
    int received = 0, loop = 0;
    int length = 0, total_read = 0;
    bool receiving = false;
    uint8_t topic_ID = 255;
    std::chrono::time_point<std::chrono::steady_clock> start, end;
    std::thread receiver_thread;

    //Timesync
    std::shared_ptr<TimeSync> timesync;
    rclcpp::TimerBase::SharedPtr timesync_timer;
    rclcpp::TimerBase::SharedPtr delay_timer;

    //Publishers
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr delay_publisher;
@[for topic in topics_send.keys()]@
    rclcpp::Publisher<px4_msgs::msg::@(topic)>::SharedPtr @(topic)_publisher;
@[end for]@

    //Subscribers
@[for topic in topics_receive.keys()]@
    rclcpp::Subscription<px4_msgs::msg::@(topic)>::SharedPtr @(topic)_subscriber;
@[end for]@

    //Subscription Callbacks
@[for topic in topics_receive.keys()]@
    void Process_@(topic)(px4_msgs::msg::@topic::SharedPtr msg);
@[end for]@

    //Topic/ID map
    std::map<std::string, int> IDmap={
@[for key, ID in topics_all.items() ]@
        {"@key",@ID},
@[end for]@
    };
};

#endif //MICRORTPS_AGENT_H
