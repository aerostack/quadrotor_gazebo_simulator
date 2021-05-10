# Motor Speed controller

# Subscribed topics

- **ground_truth/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle.

- **ground_truth/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))     
Current speed of the vehicle.

- **actuator_command/roll_pitch_yaw_rate_thrust** ([mav_msgs/RollPitchYawrateThrust](http://docs.ros.org/api/mav_msgs/html/msg/RollPitchYawrateThrust.html))           
Actuator command for the multirotor specifying roll (rad), pitch (rad), yaw rate (rad/s) and thrust (N: Newtons).

# Published topics

- **actuator_command/motor_speed** ([mav_msgs/Actuators](http://docs.ros.org/api/mav_msgs/html/msg/Actuators.html))           
Motor speeds of multirotor.

----
# Contributors

**Code maintainer:** Alberto Rodelgo