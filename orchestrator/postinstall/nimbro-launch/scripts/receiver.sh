#!/bin/bash

N=${1:-3}
PORT_BASE=${2:-17000}
LOCAL_ROBOT_NAMESPACE=${3:-tb3}
PROTOCOL=${4:-tcp}

LAUNCH_DIR=$(rospack find nimbro-simulator)/launch 
LAUNCH_FILE=$LAUNCH_DIR/receivers.launch

echo "<launch>" > $LAUNCH_FILE
for ((i=0; i<N; i++)); do
    ns=${LOCAL_ROBOT_NAMESPACE}_$i
    port=$(($PORT_BASE + $i))
    receiver_name=${PROTOCOL}_receiver$i
    protocol=$PROTOCOL
    echo -e '\t<include file="$(find nimbro-simulator)/launch/incoming.launch" >'
    echo -e '\t\t<arg name="ns" value="'$ns'"/>'
    echo -e '\t\t<arg name="port" value="'$port'" />'
    echo -e '\t\t<arg name="receiver_name" value="'$receiver_name'" />'
    echo -e '\t\t<arg name="protocol" value="'$protocol'" />'
    echo -e "\t</include>"
done >> $LAUNCH_FILE
echo -e "</launch>" >> $LAUNCH_FILE