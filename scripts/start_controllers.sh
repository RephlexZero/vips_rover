#!/bin/bash

# Controller startup script for rover simulation
# This script waits for the controller manager to be ready and then loads and starts the controllers

echo "Waiting for controller_manager to be ready..."

# Function to check if service is available
wait_for_service() {
    local service_name=$1
    local timeout=30
    local count=0
    
    while ! ros2 service list | grep -q "$service_name" && [ $count -lt $timeout ]; do
        echo "Waiting for $service_name service... ($count/$timeout)"
        sleep 1
        ((count++))
    done
    
    if [ $count -ge $timeout ]; then
        echo "ERROR: $service_name service not available after $timeout seconds"
        return 1
    fi
    
    echo "$service_name service is available"
    return 0
}

# Wait for controller manager services
wait_for_service "/controller_manager/load_controller"
if [ $? -ne 0 ]; then
    exit 1
fi

echo "Controller manager is ready! Loading controllers..."

# Load and configure joint_state_broadcaster
echo "Loading joint_state_broadcaster..."
ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: joint_state_broadcaster}"

echo "Configuring joint_state_broadcaster..."
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: joint_state_broadcaster}"

echo "Starting joint_state_broadcaster..."
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{start_controllers: [joint_state_broadcaster], stop_controllers: [], strictness: 1}"

# Load and configure ackermann_steering_controller
echo "Loading ackermann_steering_controller..."
ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: ackermann_steering_controller}"

echo "Configuring ackermann_steering_controller..."
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: ackermann_steering_controller}"

echo "Starting ackermann_steering_controller..."
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{start_controllers: [ackermann_steering_controller], stop_controllers: [], strictness: 1}"

echo "All controllers loaded and started successfully!"

# Show controller status
echo "Controller status:"
ros2 control list_controllers

echo "Controllers are ready for use!"
