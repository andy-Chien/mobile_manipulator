# Mobile_Manipulator

## Getting Started
1. Clone this repo including submodules:
   ```
   git clone https://github.com/NanyangBot/Mobile_Manipulator.git
   git submodule update --init
   ```
2. Install dependent packages and build
   ```
   rosdep init # if you never did this
   rosdep update
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ros2 run flexbe_app nwjs_install # may take some time
   ```
   
## Note
1. Checkout to the branch for each component first before modification
   ```
   # cd to this repo directory
   cd <path-to-this-repo>
   
   # check out to your branch, like TC1, TC2, TC3, ...
   git checkout <your branch>
   ```
   
2. If you want to follow the updates on the main branch, please merge main into yours
   ```
   # after checkout to your branch
   git merge main
   ```
   
3. Custom defined msgs should be placed in mm_msgs package


## How to run
1. Run MVP1 Demo:
   
   in separate terminals, after sourcing install/setup.bash
   ```
   ros2 run vision_preprocess_srv vision_prepro_srv
   ros2 run wall_painting_trajectory_planner start
   ros2 launch mm_flexbe_behaviors mvp_1_demo.launch.py
   ```

3. Run MoveIt and Nav2 with trajectory controller in Gazebo:
   ```
   ros2 launch mm_description gazebo_trajectory.launch.py
   
   # launch Nav2 but don't load it's controller server, we use our own.
   ros2 launch mm_navigation_config robot_navigation.launch.py use_sim_time:=true launch_controller:=false
   
   # launch MoveIt.
   ros2 launch mm_moveit_config robot_moveit.launch.py use_sim_time:=True
   
   # run this node to merge and adapt planned path and arm trajectory from Nav2 and MoveIt
   ros2 launch mm_cmd_adapter cmd_adapter.launch.py
   ```
   
4. Run servo controller with xbox joystick in Gazebo:
   ```
   # make sure the port of joystick is correct, something like js0, js1
   ros2 launch mm_description gazebo_servo.launch.py joy_dev:=<js port>
   ```
   For other joysticks can write a config file to define behavior of each axis and button,
   please follow mm_joystick/config/xbox_joy.yaml to create your own, and use following command when launching.
   ```
   # add the name of joysitck config file like xxxx.yaml
   ros2 launch mm_description gazebo_servo.launch.py joy_dev:=<js port> joy_config_file:=<file name>
   ```
   
3. Run mobile path controller in Gazebo:
   ```
   ros2 launch mm_description gazebo_path.launch.py
   
   # launch Nav2 but don't load it's controller server, we use our own.
   ros2 launch mm_navigation_config robot_navigation.launch.py use_sim_time:=true launch_controller:=false
   
   # run this node to adapt planned path from Nav2 to controller 
   ros2 launch mm_cmd_adapter cmd_adapter.launch.py
   ```

4. Run FlexBE UI:
   ```
   ros2 launch flexbe_app flexbe_full.launch.py
   ```
### Note
Using fake hardware instead of Gazebo
```
gazebo_servo.launch.py      -> mm_servo_fakehardware.launch.py
gazebo_trajecotry.launch.py -> mm_trajec_fakehardware.launch.py
gazebo_path.launch.py       -> mm_path_fakehardware.launch.py
```
