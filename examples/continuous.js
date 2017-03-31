/*

Continuous - taking a continuous range reading.

*/
var tessel = require('tessel');
var VL53LOX = require('../index');

var _vl53l0x = new VL53LOX(tessel.port.A);

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