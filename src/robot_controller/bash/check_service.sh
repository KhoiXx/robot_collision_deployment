#!/bin/bash

SERVICE_NAME="/register_robot"

if rosservice list | grep -q "$SERVICE_NAME"; then
    echo "Service $SERVICE_NAME already exists. Skipping node launch."
else
    echo "Service $SERVICE_NAME does not exist. Starting node."
    rosrun my_package my_service_node
fi