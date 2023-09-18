# BotOnAWire

## Packages
### Active
* Wire-Bot: The main control node. Only a beginning. 
* Position-Estimator: Fuses GPS and encoder information to track position on the cable

### Need sorting
* server
* web
* sound: manages the speakers (this might go on the peripheral microcontroller?)
* arduino-com: I think this is obsolete, but perhaps there are settings in it?
* All the files that are related to AI/CV bird detection need to be put into their own package

### Obsolete
* `boaw_sm.py`: The original main file, with a state machine built from `smach`. `smach` is too cumbersome; better to create our own 

## Todo
* (Kunal, w/Sri): Implement CV pipeline on Orin Nano
  * Create ROS package that manages CV interface
  * Define message data: (bird/no bird); confidence; window (estimate distance?); count?
  * Test pipeline from camera to Nano
  * Add second camera
* (Puen, with Greg and Sri): Rebuild ROS
  * Generate structure, packages
  * Implement basic behavior on "desktop" version of robot
