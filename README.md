# eyantra_ROS

E-yantra Robotics competition 2016-17 (Explorer Bot Theme)

The package contains the following folder of our concern:

    launch :

        move_base.launch - This launches the MoveBaseAction Server and loads the world map

        rviz_navigation.launch - This Launches Rviz simulator

        

        include folder : 

            move_base.launch.xml - This launches all the parameter files and the Trajectory planners

            

    Maps : 

        mars_1.yaml - This parameter file contains the parameters for the world map

    

    Param :

        base_local_planner_params - Prameters for Base Local Planners

        costmap_common_params - This contains common parameters for local and global costmap

        global_costmap_params - parameters for global costmap

        local_costmap_params - parameters for local costmap

        move_base_params - parameters for MoveBaseAction server

        navfn_global_planner_params - global planner parameters

        

    Scripts : 

        move.py - This contains the whole algorithm for our movement and task implementation

        odometry_calc.py - This contains the odometry info to move the robot

        

Martian.ino - The arduino code for our robot
