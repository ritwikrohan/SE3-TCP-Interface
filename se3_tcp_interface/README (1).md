# Assignment 8

## cartesian_motion_controller

It obtains tool and target poses from the sensor and moves the robot to the target. Config file looks like this

```yaml
controller_manager:
  ros__parameters:
    update_rate: 20 

    cartesian_motion_controller:
        type: assignment9_controller/CartesianMotionController
        
cartesian_motion_controller:
   ros__parameters:
      joints:
         - shoulder_pan_joint
         - shoulder_lift_joint
         - elbow_joint
         - wrist_1_joint
         - wrist_2_joint
         - wrist_3_joint
      robot_base_link: base
      end_effector_link: flange
      sensor_link: sensor_frame
      sensor_poses:                  
         tool_pose_name: tool_sensor
         target_pose_name: target_sensor
      tool_link: tool_frame
      interface_name: velocity
