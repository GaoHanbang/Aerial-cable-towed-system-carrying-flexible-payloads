//
// Created by Damien SIX on 15/06/2021.
//

@###############################################
@#
@# EmPy template for generating deserialize_wrapper.h file
@#
@###############################################

#ifndef DESERIALIZE_WRAPPER_H
#define DESERIALIZE_WRAPPER_H

#include <fastcdr/Cdr.h>
@[for head in topics_head]@
@[if head !="timesync"]@
#include "px4_msgs/msg/@(head).hpp"
@[end if]@
@[end for]@
#include "px4_msgs/msg/timesync.hpp"

@[for topic in topics_all.keys()]@
    void deserialize(px4_msgs::msg::@(topic)& msg, eprosima::fastcdr::Cdr& cdr_des);
    void serialize(px4_msgs::msg::@(topic)& msg, eprosima::fastcdr::Cdr& scdr);
@[end for]@

#endif