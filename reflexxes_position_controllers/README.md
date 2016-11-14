Reflexxes Effort Controllers
============================

Example rosparam controller configuration:

```yml
reflexxes_controller:
    type: reflexxes_position_controllers/JointPositionController
    recompute_trajectory: false
    joint_names: 
        - lwr_0_joint
        - lwr_1_joint
        - lwr_2_joint
        - lwr_3_joint
        - lwr_4_joint
        - lwr_5_joint
        - lwr_6_joint
    joints:
        lwr_0_joint:
            position_tolerance: 0.2
            max_acceleration: 0.5
            max_velocity: 0.24
        lwr_1_joint:
            position_tolerance: 0.2
            max_acceleration: 0.5
            max_velocity: 0.24
        lwr_2_joint:
            position_tolerance: 0.2
            max_acceleration: 0.5
            max_velocity: 0.24
        lwr_3_joint:
            position_tolerance: 0.2
            max_acceleration: 0.5
            max_velocity: 0.24
        lwr_4_joint:
            position_tolerance: 0.2
            max_acceleration: 0.5
            max_velocity: 0.24
        lwr_5_joint:
            position_tolerance: 0.2
            max_acceleration: 0.5
            max_velocity: 0.24
        lwr_6_joint:
            position_tolerance: 0.2
            max_acceleration: 0.5
            max_velocity: 0.24
```
