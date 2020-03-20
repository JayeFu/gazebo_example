# How to use
1. To launch mbx for display in Rviz

   ```bash
   roslaunch gazebo_example display_mbx_in_rviz.launch
   ```

2. To launch mbx for simulation in Gazebo

   ```bash
   roslaunch gazebo_example display_mbx.launch
   ```

    then if needs to move the UR-10 arm, either
    ```bash
    rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
    ```
    or
    ```bash
    rosrun gazebo_example move_arm.py
    ```

3. To see drone in Rviz

    first please change directory to `gazebo_example/urdf`, then run the command line below
    ```bash
    roslaunch urdf_tutorial display.launch model:=drone.urdf.xacro
    ```

4. To launch drone for simulation in Gazebo

   ```bash
   roslaunch gazebo_example display_drone.launch
   ```

   Then if needs to move the homemade simple mechanism, please

   ```bash
   rosrun gazebo_example move_drone.py
   ``` 
