Sunflower Controller Package
===

Provides a mid-level, unified control system for the Sunflower robot.
Creates an ActionServer on sf_controller and acts as an interface with the low level control packages.

##Configuration

The ```config/joints``` directory contains the yaml config for joints named ```{common_controllers}.yaml```.  This details the configuration settings for each servo.  The min and max positions will be used by the controller to trim any commands sent to a valid position.  For example, if the max is 500 and the max sent is 600, then the motor will only recieve 500.  The init value will be mapped to the 0 point by the controller.  All commands are accepted in radians and automatically converted to ticks.

The ```config/joints/named_positions.yaml``` files give named positions for known joints, in radians.


##Launching

The ```start_controller.launch``` launches the controller and dynamixel servos.
