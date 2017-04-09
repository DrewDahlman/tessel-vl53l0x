Tessel-VL53L0X
====================

An API wrapper for interfacing with the VL53L0X time of flight sensor and a Tessel. Returns distance in mm. 

## Install
`npm install tessel-vl53l0x`

## Use
<pre>
var VL53L0X = require('tessel-vl53l0x');
var _vl53l0x = new VL53L0X(tessel.port.A);

/*
------------------------------------------
| Setting a lower limit increases the potential range of the sensor but also
| seems to increase the likelihood of getting an inaccurate reading because of
| unwanted reflections from objects other than the intended target.
| Defaults to 0.25 MCPS as initialized by the ST API and this library.
------------------------------------------ */
_vl53l0x.setSignalRateLimit(.05, () => {
	_vl53l0x.startCapture();			
});

/*
------------------------------------------
| Listen for the sensor to emit 'distance'
------------------------------------------ */
_vl53l0x.on('distance', function(data){
	console.log(data);
});
</pre>

There are two options single capture and continuous. You can see sample code in the examples. `startCapture()` & `singleCapture()`

## Configuration
You can change a few basic configurations on the sensor. The best one to tweak will be the `setSignalRateLimit`. Setting a lower limit increases the potential range of the sensor but also seems to increase the likelihood of getting an inaccurate reading because of unwanted reflections from objects other than the intended target.

## Wiring
![Wiring Diagram](https://github.com/DrewDahlman/tessel-vl53l0x/blob/master/wiring-diagram.png?raw=true)



