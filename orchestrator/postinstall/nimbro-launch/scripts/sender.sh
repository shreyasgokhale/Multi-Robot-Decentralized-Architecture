#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
N=${1:-3}
ROBOT_TARGET_PREFIX=${2:-robot-nimbro-}
LOCAL_ROBOT_NAMESPACE=${3:-tb3}
PROTOCOL=${4:-tcp}

LAUNCH_DIR=$(rospack find nimbro-simulator)/launch 
LAUNCH_FILE=$LAUNCH_DIR/senders.launch

echo "<launch>" > "$LAUNCH_FILE"
for ((i=0; i<N; i++)); do
    target=$ROBOT_TARGET_PREFIX$i
    ns=${LOCAL_ROBOT_NAMESPACE}_$i
    port=17000
    sender_name=${PROTOCOL}_sender$i
    protocol=$PROTOCOL
    echo -e '\t<include file="$(find nimbro-simulator)/launch/outgoing.launch" >'
    echo -e '\t\t<arg name="target" value="'$target'" />'
    echo -e '\t\t<arg name="target_port" value="'$port'" />'
    echo -e '\t\t<arg name="ns" value="'$ns'" />'
    echo -e '\t\t<arg name="sender_name" value="'$sender_name'" />'
    echo -e '\t\t<arg name="protocol" value="'$protocol'" />'
    echo -e "\t</include>"
done >> "$LAUNCH_FILE"
echo -e "</launch>" >> "$LAUNCH_FILE"