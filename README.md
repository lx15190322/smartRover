# smartRover
This project can be used as a combined work with armTracking for two-robot delivery task, but can also run independently for redball detecting and tracking task of the rover.

'talker3.py' is the ros node for vision detect and decision making of the rover. 

'smartcar_copy.ino' is the controller for Arduino, 'Adafruit_PWMServoDriver.h' and 'Adafruit_PWMServoDriver.cpp' are the API for servo motors of the wheels.

1. For independent implementation, type the following commands in the terminal:
roscore
rosrun <-Your Package Name-> talker3.py
rosrun rosserial_python serial_node.py dev/ttyACM0 _baud:=9600

2. For multi-robot cooperation:
*There is a official bug (node conflict) in rosserial_python package when running multiple Arduino board under multi-machine (ssh) condition of ros system, this can be solved by change the node name in serial_node.py.
Same ros commands to 1. With additional ssh settings, see the following links:
(1)MultipleMachines tutorial: http://wiki.ros.org/ROS/Tutorials/MultipleMachines
(2)Supplement steps for (1): https://github.com/lx15190322/ArmTracking/tree/master/README
