# setpoint_buffer
Here is the first versions of the translated files (still some problems though) 
Here you will find three files required to start the drone: 
- setpoint_buffer.py -> main file -> run python3 setpoin_buffer.py to start everything 
- guidance_library.py -> calls attitude and is called in setpoint_buffer (still has some issues though) 
- attitude_library.py 

NOTE: There might be some slight issues with the translations from python/ros1 -> python/ros2 
NOTE: The main issue is that when calling: 
```
ros2 service call /rogx_vision_mockup/mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: True}' 
```
it is stuck there without any responce. Previously it was not the issue and calling was running smoothly
