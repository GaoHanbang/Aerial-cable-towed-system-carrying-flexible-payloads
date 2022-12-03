#!/usr/bin/env python3
# Author Damien SIX 16/06/2021
# Partially copied from PX4 project

import sys
import os
from uorb_rtps_classifier import Classifier
import px_generate_uorb_topic_helper
import argparse

try:
    from six.moves import input
except ImportError as e:
    print("Failed to import six: " + e)
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user six")
    print("")
    sys.exit(1)

try:
    from packaging import version
except ImportError as e:
    print("Failed to import packaging: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user packaging")
    print("")
    sys.exit(1)

try:
    import em
except ImportError as e:
    print("Failed to import em: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user empy")
    print("")
    sys.exit(1)

try:
    import genmsg.template_tools
except ImportError as e:
    print("Failed to import genmsg: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyros-genmsg")
    print("")
    sys.exit(1)


def check_rtps_id_uniqueness(classifier):
    """
    Checks if there are no ID's for different msgs repeated on the map
    """

    repeated_ids = dict()

    full_send_list = dict(list(msg for msg in list(classifier.msgs_to_send.items(
    ))) + list(list(msg[0].items())[0] for msg in classifier.alias_msgs_to_send))
    full_receive_list = dict(list(msg for msg in list(classifier.msgs_to_receive.items(
    ))) + list(list(msg[0].items())[0] for msg in classifier.alias_msgs_to_receive))
    full_ignore_list = dict(list(msg for msg in list(classifier.msgs_to_ignore.items(
    ))) + list(list(msg[0].items())[0] for msg in classifier.alias_msgs_to_ignore))

    # check if there are repeated ID's on the messages to send
    for key, value in list(full_send_list.items()):
        if list(full_send_list.values()).count(value) > 1:
            repeated_ids.update({key: value})

    # check if there are repeated ID's on the messages to receive
    for key, value in list(full_receive_list.items()):
        if list(full_receive_list.values()).count(value) > 1:
            repeated_ids.update({key: value})

    # check if there are repeated ID's on the messages to ignore
    for key, value in list(full_ignore_list.items()):
        if list(full_ignore_list.values()).count(value) > 1:
            repeated_ids.update({key: value})

    # check if there are repeated IDs between classified and unclassified msgs
    # check send and ignore lists
    send_ignore_common_ids = list(set(full_ignore_list.values(
    )).intersection(list(full_send_list.values())))
    for item in list(full_send_list.items()):
        for repeated in send_ignore_common_ids:
            if item[1] == repeated:
                repeated_ids.update({item[0]: item[1]})
    for item in list(full_ignore_list.items()):
        for repeated in send_ignore_common_ids:
            if item[1] == repeated:
                repeated_ids.update({item[0]: item[1]})

    # check receive and ignore lists
    receive_ignore_common_ids = list(set(full_ignore_list.values(
    )).intersection(list(full_receive_list.values())))
    for item in list(full_receive_list.items()):
        for repeated in receive_ignore_common_ids:
            if item[1] == repeated:
                repeated_ids.update({item[0]: item[1]})
    for item in list(full_ignore_list.items()):
        for repeated in receive_ignore_common_ids:
            if item[1] == repeated:
                repeated_ids.update({item[0]: item[1]})

    all_msgs = {}
    all_msgs.update(full_send_list)
    all_msgs.update(full_receive_list)
    all_msgs.update(full_ignore_list)
    all_ids = list(all_msgs.values())
    all_ids.sort()

    if not repeated_ids:
        print("All good. RTPS ID's are unique")
    else:
        raise AssertionError(", ".join('%s' % msgs for msgs in list(repeated_ids.keys())) +
                             " have their ID's repeated. Please choose from the following pool:\n" +
                             ", ".join('%d' % id for id in px_generate_uorb_topic_helper.check_available_ids(all_ids)))


parser = argparse.ArgumentParser()
parser.add_argument("-t", "--topic-msg-dir", dest='msgdir', type=str,
                    help="Topics message, by default using relative path 'msg/'", default="msg")
parser.add_argument("-o", "--file-out-dir", dest='outdir', type=str,
                    help="File output folder", default="out")
parser.add_argument("-c", "--config-file", dest='configfile', type=str,
                    help="yaml configuration file", default="")
args = parser.parse_args()

agent_out_dir = os.path.abspath(args.outdir)
yaml_id_file = os.path.abspath(args.configfile)
msg_dir = os.path.abspath(args.msgdir)

# parse yaml file into a map of ids
classifier = Classifier(os.path.abspath(yaml_id_file), msg_dir)

# check if there are no ID's repeated
check_rtps_id_uniqueness(classifier)


uRTPS_AGENT_CPP_TEMPL_FILE = os.path.abspath('templates/microRTPS_agent.cpp.em')
uRTPS_AGENT_H_TEMPL_FILE = os.path.abspath('templates/microRTPS_agent.h.em')
uRTPS_AGENT_CPP_DW_TEMPL_FILE = os.path.abspath('templates/deserialize_wrapper.cpp.em')
uRTPS_AGENT_H_DW_TEMPL_FILE = os.path.abspath('templates/deserialize_wrapper.h.em')


def generate_from_template(template_file, output_file, em_globals):
    # Generate files from em
    ofile = open(output_file, 'w')
    interpreter = em.Interpreter(output=ofile, globals=em_globals)

    try:
        interpreter.file(open(template_file))
    except OSError:
        ofile.close()
        os.remove(output_file)
        raise
    interpreter.shutdown()
    ofile.close()


def get_message_field(filename):
    # Get messages fields
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(
        "px4_msgs", os.path.basename(filename))
    spec = genmsg.msg_loader.load_msg_from_file(
        msg_context, filename, full_type_name)
    field_names = {}
    for field in spec.parsed_fields():
        field_names.update({field.name: field.type})
    return field_names


def generate_agent(out_dir):
    topics_send = classifier.msgs_to_send
    for m in classifier.alias_msgs_to_send:
        topics_send.update(m[0])
    topics_receive = classifier.msgs_to_receive
    for m in classifier.alias_msgs_to_receive:
        topics_receive.update(m[0])
    topics_head = set()
    topics_fields = {}
    for topic in list(topics_send.keys()) + list(topics_receive.keys()):
        topics_head.add(''.join([ ("_" + z.lower()) if z.isupper() else z for z in topic[0].lower() + topic[1:]]))
        if topic not in topics_fields:
            topics_fields[topic] = get_message_field(os.path.join(msg_dir, topic+".msg"))
    topics_all = {}
    for m in topics_send:
        topics_all[m] = topics_send[m]
    for m in topics_receive:
        if m not in topics_all:
            topics_all[m] = topics_receive[m]

    generate_from_template(uRTPS_AGENT_H_TEMPL_FILE,
                           os.path.join(agent_out_dir, "microRTPS_agent.h"),
                           {"topics_all": topics_all,
                            "topics_send": topics_send,
                            "topics_receive": topics_receive,
                            "topics_head": topics_head})

    generate_from_template(uRTPS_AGENT_CPP_TEMPL_FILE,
                           os.path.join(agent_out_dir, "microRTPS_agent.cpp"),
                           {"topics_all": topics_all,
                            "topics_send": topics_send,
                            "topics_receive": topics_receive,
                            "topics_head": topics_head})

    generate_from_template(uRTPS_AGENT_H_DW_TEMPL_FILE,
                           os.path.join(agent_out_dir, "deserialize_wrapper.h"),
                           {"topics_all": topics_all,
                            "topics_head": topics_head})

    generate_from_template(uRTPS_AGENT_CPP_DW_TEMPL_FILE,
                           os.path.join(agent_out_dir, "deserialize_wrapper.cpp"),
                           {"topics_all": topics_all,
                            "topics_fields": topics_fields})

generate_agent(agent_out_dir)
print(("\nAgent created in: " + agent_out_dir))
