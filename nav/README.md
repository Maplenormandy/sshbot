Navigation
===================

Stores and updates map and pose estimate information.  Also manages min-cost queries using a service that takes an array of objects to visit (in order). TO run, simply use

    roslaunch nav move_base.launch

Current features are:



Planned features:



Things to adjust:
launch file
costmap_common_params.yaml


Topics:
    Out:
        goal_cost (goal_dist may not reflect our progress in achieving the goal pose)
        cmd_vel
    In:
        ball_pos
        laser_scan
        
Service
    path_order(array of strings)
    botclient_map(string)




