# Wire-Bot

Manages the control of the robot.

## Functionality
* Monitors sensor inputs to determine bird presence
* Sets target speed, including calling a speed PID control service
* Engages deterrents

## Messages
### In
* activation: to set the state from IDLE to PATROL_FWD if true, otherwise set to IDLE
* /distance/front: distance to objects in the front in cm

### Out
* target_speed: target speed of the robot in meters per second
* led_cmd: number of flashes for the led to blink 

## Services
### Service Proxy
* speed_control: creates service request for a target speed after PID control


# distance-estimator

Estimates the distance to both the front and the back

## Functionality
* Receives sensor data from all four distance sensor
* Returns distance after sensor fusion

## Messages
### In
* /rangefinder/fore/MB: distance from the front ultrasonic rangefinder
* /rangefinder/aft/MB: distance from the back ultrasonic rangefinder
* /rangefinder/fore/TF: distance from the front TF rangefinder
* /rangefinder/aft/TF: distance from the back TF rangefinder

### Out
* /distance/fore: distance to the front
* /distance/aft: distance to the back


# speed_controller

Manages the PID control of speed

## Functionality
* Receives target speed and current speed
* Implements PID control and calculates a new target speed

## Services
### Service
* speed_control: provides PID control service to control the motor speed