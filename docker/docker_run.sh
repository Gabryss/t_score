#!/bin/bash
#xhost +local:docker
LEOROVER='leo05'

# Note: replace --restart=always to --rm for testing
docker run -it \
    --rm \
    --network=host \
    --ipc=host \
    --pid=host \
    --privileged \
    --env UID=$(id -u) \
    --env GID=$(id -g) \
    --env ROS_NAMESPACE=${ROS_NAMESPACE} \
    --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    --volume="${PWD}/../ros2_ws/src/t_score:/root/ros2_ws/src/t_score" \
    --volume="${PWD}/data:/root/ros2_ws/data" \
    --name roughness \
    local/t_score:humble \
    ros2 launch t_score t_score_launch.py
