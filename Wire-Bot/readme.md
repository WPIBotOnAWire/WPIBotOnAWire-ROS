# Wire-Bot

Manages the control of the robot.
## Functionality
* Monitors sensor inputs do determine bird presence
* Monitors battery state
* Determines speed
* Engages deterrents

## Messages
### In
* sensor info from SAMD21 (see Arduino code for specifics)
* TBD: classification data from CV module

### Out
* target_speed: target speed of the robot in meters per second
* TBD: deterrent commands