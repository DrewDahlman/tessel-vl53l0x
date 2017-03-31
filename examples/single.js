/*

Single - An example of taking a single range reading.

*/
var tessel = require('tessel');
var VL53LOX = require('../index');

var _vl53l0x = new VL53LOX(tessel.port.A);

_vl53l0x.on('distance', function(data){
	console.log(data);
});

setTimeout( function(){
	_vl53l0x.singleCapture();
},2500);