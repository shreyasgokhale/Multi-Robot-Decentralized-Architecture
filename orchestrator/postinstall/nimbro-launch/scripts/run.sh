#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

ROBOT_COUNT=${1:-3}
PORT_BASE=${2:-17000}
ROBOT_TARGET_PREFIX=${3:-robot-nimbro-}
LOCAL_ROBOT_NAMESPACE=${4:-tb3}
PROTOCOL=${5:-tcp}

SCRIPT_DIR=$(rospack find nimbro-simulator)/scripts 
"$SCRIPT_DIR"/sender.sh "$ROBOT_COUNT" "$ROBOT_TARGET_PREFIX" "$LOCAL_ROBOT_NAMESPACE" "$PROTOCOL"
"$SCRIPT_DIR"/receiver.sh n="$ROBOT_COUNT" "$PORT_BASE" "$LOCAL_ROBOT_NAMESPACE" "$PROTOCOL"

roslaunch nimbro-simulator simulator_nimbro.launch