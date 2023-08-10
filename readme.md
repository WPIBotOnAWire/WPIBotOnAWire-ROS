# BotOnAWire

## Packages
### Active
* Wire-Bot: The main control node. Only a beginning.
* Position-Estimator: Fuses GPS and encoder information to track position on the cable

### Need sorting
* server
* web
* sound: manages the speakers
* arduino-com: I think this is obsolete, but perhaps there are settings in it?
* All the files that are related to AI/CV bird detection need to be put into their own package

### Obsolete
* `boaw_sm.py`: The original main file, with a state machine built from `smach`. `smach` is too cumbersome; better to create our own 