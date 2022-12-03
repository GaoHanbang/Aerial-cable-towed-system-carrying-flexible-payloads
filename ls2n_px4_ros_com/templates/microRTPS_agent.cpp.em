//
// Created by Damien SIX on 15/06/2021.
// Parts of the code are copied from https://github.com/PX4/px4_ros_com
//

@###############################################
@#
@# EmPy template for generating microRTPS_agent.cpp file
@#
@###############################################

#include "microRTPS_agent.h"
#include <unistd.h>
#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>
#include "rclcpp/serialization.hpp"
#include "deserialize_wrapper.h"

using namespace std::chrono_literals;

MicroRtpsAgent::MicroRtpsAgent() : Node("micrortps_agent") {
    // Node parameters
    declare_parameter<std::string>("transport", "UDP");
    declare_parameter<uint16_t>("receive_port", DEFAULT_RECV_PORT);
    declare_parameter<uint16_t>("send_port", DEFAULT_SEND_PORT);
    declare_parameter<std::string>("device", DEVICE);
    declare_parameter<int64_t>("baudrate", BAUDRATE);
    declare_parameter<std::string>("ip_address", DEFAULT_IP);

    std::string transport;
    std::string device;
    uint16_t rec;
    uint16_t sen;
    std::string ip;
    int64_t baudrate;
    if(get_parameter("transport", transport)) {
        if (transport == "UDP") {
            options.transport = Options::eTransports::UDP;
            if(get_parameter("receive_port", rec))
                options.recv_port = rec;
            if(get_parameter("send_port", sen))
                options.send_port = sen;
            if(get_parameter("ip_address", ip))
                strcpy(options.ip, ip.c_str());
        }
        if (transport == "UART") {
            options.transport = Options::eTransports::UART;
            if(get_parameter("baudrate", baudrate))
                options.baudrate = baudrate;
            if(get_parameter("device", device))
                strcpy(options.device, device.c_str());
        }
    }

    RCLCPP_INFO(get_logger(), "Starting MicroRTPS Agent");
    switch (options.transport) {
        case Options::eTransports::UART:
        {
            transport_node = new UART_node(options.device, options.baudrate, options.poll_ms,
                                           options.sw_flow_control, options.hw_flow_control, options.verbose_debug);
            RCLCPP_INFO(get_logger(), "UART transport: device: %s; baudrate: %ld; sleep: %dus; poll: %dms; flow_control: %s\n",
                        options.device, options.baudrate, options.sleep_us, options.poll_ms,
                        options.sw_flow_control ? "SW enabled" : (options.hw_flow_control ? "HW enabled" : "No"));
        }
            break;
        case Options::eTransports::UDP:
        {
            transport_node = new UDP_node(options.ip, options.recv_port, options.send_port, options.verbose_debug);
            RCLCPP_INFO(get_logger(), "UDP transport: ip address: %s; recv port: %u; send port: %u; sleep: %dus\n",
                        options.ip, options.recv_port, options.send_port, options.sleep_us);
        }
            break;
        default:
            RCLCPP_INFO(get_logger(), "EXITING...");
            return;
    }

    if (0 > transport_node->init())
    {
        RCLCPP_INFO(get_logger(), "EXITING...");
        return;
    }
    sleep(1);
    connected = true;

    // Publishers
    auto qos_px4_com = rclcpp::SensorDataQoS();
    qos_px4_com.keep_last(1);
    delay_publisher = create_publisher<std_msgs::msg::UInt64>("RTPSBridgeDelay", qos_px4_com);
@[for topic in topics_send.keys()]@
    @(topic)_publisher = create_publisher<px4_msgs::msg::@topic>("@(topic)_PubSubTopic", qos_px4_com);
@[end for]@

    // Subscritption
@[for topic in topics_receive.keys()]@
    @(topic)_subscriber = create_subscription<px4_msgs::msg::@topic>("@(topic)_PubSubTopic",
                                                qos_px4_com,
                                                std::bind(&MicroRtpsAgent::Process_@(topic),
                                                this, std::placeholders::_1));
@[end for]@

    // Init timesync
    timesync = std::make_shared<TimeSync>(options.verbose_debug, Timesync_publisher);
    timesync_timer = create_wall_timer(100ms, [this] { timesync->spin(); });

    // Delat pulisher timer
    delay_timer = create_wall_timer(1s, [this] { MeanDelayPublish(); });

    running = true;
    // Receiver thread
    receiver_thread = std::thread(&MicroRtpsAgent::Receive, this);
    receiver_thread.detach();
}

    //Subscription Callbacks
@[for topic in topics_receive.keys()]@
void MicroRtpsAgent::Process_@(topic)(px4_msgs::msg::@topic::SharedPtr msg){
@[if topic == "Timesync"]@
if (msg->sys_id == 0) {
@[end if]@
    uint32_t length = 0;
    char data_buffer[BUFFER_SIZE] = {};
    uint8_t topic_ID = IDmap["@topic"];

    size_t header_length = transport_node->get_header_length();
    /* make room for the header to fill in later */
    eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[header_length], sizeof(data_buffer)-header_length);
    eprosima::fastcdr::Cdr scdr(cdrbuffer);

    // apply timestamps offset
    uint64_t timestamp = getMsgTimestamp(msg.get());
    uint64_t timestamp_sample = getMsgTimestampSample(msg.get());
    timesync->addOffset(timestamp);
    setMsgTimestamp(msg.get(), timestamp);
    timesync->addOffset(timestamp_sample);
    setMsgTimestampSample(msg.get(), timestamp_sample);
    // Serialize message
    serialize(*msg.get(), scdr);
    {
        length = scdr.getSerializedDataLength();
        if (0 < (length = transport_node->write(topic_ID, data_buffer, length)))
        {
            total_sent += length;
            ++sent;
        }
    }
@[if topic == "Timesync"]@
}
@[end if]@
}
@[end for]@

void MicroRtpsAgent::MeanDelayPublish()
{
    std_msgs::msg::UInt64 msg;
    msg.data = timesync->getMeanDelay();
    delay_publisher->publish(msg);
}

void MicroRtpsAgent::Receive()
{
    while (running) {
        ++loop;
        if (!receiving) start = std::chrono::steady_clock::now();
        // Publish messages received from UART
        while (0 < (length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE))) {
            Publish(topic_ID, data_buffer, sizeof(data_buffer));
            ++received;
            total_read += length;
            receiving = true;
            end = std::chrono::steady_clock::now();
        }

        if ((receiving &&
             std::chrono::duration<double>(std::chrono::steady_clock::now() - end).count() > WAIT_CNST) ||
            (!running && loop > 1)) {
            std::chrono::duration<double> elapsed_secs = end - start;
            RCLCPP_INFO(get_logger(),"SENT:     %lumessages \t- %lubytes\n", (unsigned long) sent,
                   (unsigned long) total_sent);
            RCLCPP_INFO(get_logger(),"RECEIVED: %dmessages \t- %dbytes; %d LOOPS - %.03f seconds - %.02fKB/s\n",
                   received, total_read, loop, elapsed_secs.count(),
                   (double) total_read / (1000 * elapsed_secs.count()));
            received = sent = total_read = total_sent = 0;
            receiving = false;
        }
    }
}

void MicroRtpsAgent::Publish(uint8_t topic_ID, char data_buffer[], size_t len)
{
    switch (topic_ID)
    {
@[for topic in topics_send]@
        case @topics_all[topic]:
        {
            px4_msgs::msg::@(topic) msg;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            // Deserialize
            deserialize(msg, cdr_des);
@[    if topic == 'Timesync']@
            timesync->processTimesyncMsg(&msg);

            if (msg.sys_id == 1) {
@[    end if]@
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&msg);
            timesync->subtractOffset(timestamp);
            setMsgTimestamp(&msg, timestamp);
            uint64_t timestamp_sample = getMsgTimestampSample(&msg);
            timesync->subtractOffset(timestamp_sample);
            setMsgTimestampSample(&msg, timestamp_sample);
            @(topic)_publisher->publish(msg);
@[    if topic == 'Timesync']@
            }
@[    end if]@
        }
        break;
@[end for]@
        default:
            RCLCPP_WARN(get_logger(),"Unexpected topic ID '%hhu' to publish Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'", topic_ID, topic_ID);
        break;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto micro_node=std::make_shared<MicroRtpsAgent>();
    if(micro_node->connected)
        rclcpp::spin(micro_node);
    micro_node->running = false;
    sleep(0.5);
    micro_node->CloseConnection();
    rclcpp::shutdown();
    return 0;
}