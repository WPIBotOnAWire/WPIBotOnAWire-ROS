# Position-Estimator

Fuses the GPS and encoder data in a simple complementary filter. Updates position from the encoder data and then corrects it from GPS data. Untested, but the math should be close. Requires lat, lon of the end poles and droop of the cable (the algorithm accounts for the catenary shape of the cable).

## Messages
### In:
* `/encoder/meters_per_interval`: distance travelled over the last interval, in meters
* `/gps/coords`: lat, lon from GPS (not fully set up)

### Out:
* `position`: the estimated position, referenced to the center of the cable

### Todo
* Create a custom message for GPS data, including lat, lon, and some reliability data (DOP, etc.).
* Parameterize the values (wheel diameter, etc).
* Add uncertainty (à la a KF)
* Test, test, test.