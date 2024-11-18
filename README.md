# AI deck communication node

Install requirements

    pip install -r requirements.txt

# Source jazzy!

    source /opt/ros/jazzy/setup.bash

# Build package

    rosdep install -i --from-path src --rosdistro jazzy -y

    colcon build

# Run node

Connect to drone access point! and then source: 
    
    source install/setup.bash 
    
and run 
    
    ros2 run drone_streamer_pkg drone_streamer
