#include "deserialize_wrapper.h"

@[for topic in topics_all.keys()]@
void deserialize(px4_msgs::msg::@(topic)& msg, eprosima::fastcdr::Cdr& cdr_des){
@[for field in topics_fields[topic]]@
@{field_type = str(topics_fields[topic][field])}@
@[if field_type.split('[')[0].split('/')[0] == "px4_msgs"]
@{narr_type = field_type.split('[')[0].split('/')[1]}@
@{arr_num = field_type.split('[')[-1].split(']')[0]}@
        for (int i=0; i<@arr_num; i++)
            deserialize(msg.@(field)[i], cdr_des);
@[else]@
        cdr_des >> msg.@field;
@[end if]@
@[end for]@
}
void serialize(px4_msgs::msg::@(topic)& msg, eprosima::fastcdr::Cdr& scdr){
@[for field in topics_fields[topic]]@
@{field_type = str(topics_fields[topic][field])}@
@[if field_type.split('[')[0].split('/')[0] == "px4_msgs"]
@{narr_type = field_type.split('[')[0].split('/')[1]}@
@{arr_num = field_type.split('[')[-1].split(']')[0]}@
        for (int i=0; i<@arr_num; i++)
            serialize(msg.@(field)[i], scdr);
@[else]@
        scdr << msg.@field;
@[end if]@
@[end for]@
}
@[end for]@