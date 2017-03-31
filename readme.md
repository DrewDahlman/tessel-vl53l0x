Tessel-VL53L0X
====================

An API wrapper for interfacing with the VL53L0X time of flight sensor and a Tessel. Returns distance in mm. 

## Install
`npm install tessel-vl53l0x`

## Use
<pre>
var VL53LOX = require('tessel-vl53l0x');
var _vl53l0x = new VL53LOX(tessel.port.A);
</pre>

## Configuration
You can change a few basic configurations on the sensor. The best one to tweak will be the `setSignalRateLimit`. Setting a lower limit increases the potential range of the sensor but also seems to increase the likelihood of getting an inaccurate reading because of unwanted reflections from objects other than the intended target.

## Wiring
![Wiring Diagram](https://github.com/DrewDahlman/tessel-vl53l0x/blob/master/wiring-diagram.png?raw=true)



