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
   **CAUTION:** since the nodes in the launch file of display_mbx.launch is now grouped, `rqt_joint_trajectory_controller` cannot be used now! See tip 28 at [here](https://github.com/JayeFu/work_log/blob/master/work_log.md)


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
5. To launch fxw for simulation in Gazebo 
   
   ```bash
   roslaunch gazebo_example display_fwx.launch
   ```
   Then if needs to move the iiwa arm, please

   ```bash
   rosrun gazebo_example move_iiwa.launch
   ```

6. To launch all these three in Gazebo, please run
   
   ```bash
   roslaunch gazebo_example display_all.launch
   ```
   
   To further test the availability of these three, please run
   
   ```bash
   rosrun gazebo_example move_arm.py
   rosrun gazebo_example move_drone.py
   rosrun gazebo_example move_iiwa.py
   ```
   Please wait for a moment, then you will see them moving. 