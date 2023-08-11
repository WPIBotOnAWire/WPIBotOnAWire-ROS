# Wire-Bot

Manages the control of the robot.

## Functionality
* Monitors sensor inputs do determine bird presence
* Sets target speed
* Engages deterrents
* Monitors battery state

## Messages
### In
* sensor info from SAMD21 (see Arduino code for specifics)
* TBD: classification data from CV module

### Out
* target_speed: target speed of the robot in meters per second
* TBD: deterrent commands