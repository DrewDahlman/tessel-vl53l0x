"use strict"

/*

Copyright (c) 2017. All Rights Reserved.
Author: Drew Dahlman

VL53LOX API Wrapper ported to tessel. This acts as a simple wrapper for the VL53LOX Time Of Flight Sensor. 

Ported from https://github.com/pololu/vl53l0x-arduino

*/

// Constants
const tessel = require('tessel');
const Queue = require('sync-queue');
const REGISTRY = require('./registry');
const EventEmitter = require('events').EventEmitter;

class VL53LOX extends EventEmitter {

	/*
	------------------------------------------
	| constructor:void (-)
	------------------------------------------ */
	constructor( port ){
		super();

		// Assign I2C Address
		this.i2c = new port.I2C( REGISTRY.I2C_ADDR );

		// Queue system
		this.queue = new Queue();

		// Internal Variables
		this._last_status; 
		this._io_timeout = 0; // 16
		this._did_timeout = false;
		this._timeout_start_ms = Date.now(); // 16
		this._stop_variable; // 8
		this._measurement_timing_budget_us; // 32

		this._sequenceStepEnables = {
			tcc: true, 
			msrc: true,
			dss: true, 
			pre_range: true, 
			final_range: true
		};

		this._sequenceStepTimeouts = {
			pre_range_vcsel_period_pclks: 0, // 16_t
			final_range_vcsel_period_pclks: 0, // 16_t
			msrc_dss_tcc_mclks: 0, // 16_t
			pre_range_mclks: 0, // 16_t
			final_range_mclks: 0, // 16_t
      msrc_dss_tcc_us: 0, // 32_t
      pre_range_us: 0, // 32_t
      final_range_us: 0 // 32_t
		}

		// Internal Helper Methods
		this._startTimeout = () => { return this._timeout_start_ms };
		this._checkTimeoutExpired = () => { 
			return (this._io_timeout > 0 && Date.now() - this._timeout_start_ms > this._io_timeout);
		};
		this._decodeVcselPeriod = ( reg_val ) => {
			return (((reg_val) + 1) << 1);
		};
		this._encodeVcselPeriod = ( period_pclks ) => {
			return (((period_pclks) >> 1) - 1);
		};
		this._calcMacroPeriod = ( vcsel_period_pclks ) => {
			return (((2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
		};

		this._getSequenceStepTimeouts = ( callback ) => {

			// pre_range_vcsel_period_pclks
			this._readRegisters(REGISTRY.PRE_RANGE_CONFIG_VCSEL_PERIOD, 8, (err, data) => {
				this._sequenceStepTimeouts.pre_range_vcsel_period_pclks = data.readInt16BE(0);

				// msrc_dss_tcc_mclks
				this._readRegisters(REGISTRY.MSRC_CONFIG_TIMEOUT_MACROP, 8, (err, data) => {
					this._sequenceStepTimeouts.msrc_dss_tcc_mclks = data.readInt16BE(0) + 1;

					// msrc_dss_tcc_us
					this._sequenceStepTimeouts.msrc_dss_tcc_us = this.timeoutMclksToMicroseconds(this._sequenceStepTimeouts.msrc_dss_tcc_mclks, this._sequenceStepTimeouts.pre_range_vcsel_period_pclks);

					// pre_range_mclks
					this._readRegisters(REGISTRY.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 8, (err, data) => {
						this._sequenceStepTimeouts.pre_range_mclks = this.decodeTimeout( data.readInt16BE(0) );

						// pre_range_us
						this._sequenceStepTimeouts.pre_range_us = this.timeoutMclksToMicroseconds(this._sequenceStepTimeouts.pre_range_mclks, this._sequenceStepTimeouts.pre_range_vcsel_period_pclks);

							// final_range_vcsel_period_pclks
							this._readRegisters(REGISTRY.FINAL_RANGE_CONFIG_VCSEL_PERIOD, 8, (err, data) => {
								this._sequenceStepTimeouts.final_range_vcsel_period_pclks = data.readInt16BE(0);

								// final_range_mclks
								this._readRegisters(REGISTRY.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 8, (err, data) => {
									this._sequenceStepTimeouts.final_range_mclks = this.decodeTimeout(data.readInt16BE(0));

									//final_range_us
									this._sequenceStepTimeouts.final_range_us = this.timeoutMclksToMicroseconds(this._sequenceStepTimeouts.final_range_mclks, this._sequenceStepTimeouts.final_range_vcsel_period_pclks);

									callback( this._sequenceStepTimeouts );
								});
							});
					});
				});
			});
		};

	}

	/*
	------------------------------------------
	| startCapture:void (-)
	|
	| Returns a continuous reading every 33ms
	------------------------------------------ */
	startCapture(){
    this._writeRegisters(REGISTRY.SYSRANGE_START, 0x02);

    setInterval( () => {
      this._readRegisters(REGISTRY.RESULT_RANGE_STATUS, 16, (err, data) => {
        var _dis = (data.readInt16BE(8) + 10);
        this.emit('distance', _dis);
      });
    },33);
	}

	/*
	------------------------------------------
	| singleCapture:void (-)
	|
	| returns a single measurement.
	------------------------------------------ */
	singleCapture(){
		this._writeRegisters(REGISTRY.SYSRANGE_START, 0x02);

    this._readRegisters(REGISTRY.RESULT_RANGE_STATUS, 16, (err, data) => {
      var _dis = (data.readInt16BE(8) + 10);
      this.emit('distance', _dis);
    });
	}

	/*
	------------------------------------------
	| setSignalRateLimit:bool (-)
	|
	| Set the return signal rate limit check value in units of MCPS (mega counts
	| per second). "This represents the amplitude of the signal reflected from the
	| target and detected by the device"; setting this limit presumably determines
	| the minimum measurement necessary for the sensor to report a valid reading.
	|
	| Setting a lower limit increases the potential range of the sensor but also
	| seems to increase the likelihood of getting an inaccurate reading because of
	| unwanted reflections from objects other than the intended target.
	| Defaults to 0.25 MCPS as initialized by the ST API and this library.
	------------------------------------------ */
	setSignalRateLimit( limit_Mcps, callback ){
		if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

		let _data = (limit_Mcps * (1 << 7)).toFixed(7);

	  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	  this._writeRegisters( REGISTRY.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, _data, (err, data) => {
	  	if( callback ){
		  	callback()
		  }
	  });

	  return true;
	}

	/*
	------------------------------------------
	| getSignalRateLimit:void (-)
	| Get the return signal rate limit check value in MCPS
	------------------------------------------ */
	getSignalRateLimit( callback ){
	  this._readRegisters( REGISTRY.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 16, (err, data) => {
	  		let _data = (data.readInt16BE(0)) / (1 << 7);

	  		if( callback ){
		  		callback( _data );
		  	}
	  });
	}

	/*
	------------------------------------------
	| setMeasurementTimingBudget:bool (-)
	|
	| Set the measurement timing budget in microseconds, which is the time allowed
	| for one measurement; the ST API and this library take care of splitting the
	| timing budget among the sub-steps in the ranging sequence. A longer timing
	| budget allows for more accurate measurements. Increasing the budget by a
	| factor of N decreases the range measurement standard deviation by a factor of
	| sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
	| based on VL53L0X_set_measurement_timing_budget_micro_seconds()
	------------------------------------------ */
	setMeasurementTimingBudget( budget_us ){

		// SequenceStepEnables enables;
		//  SequenceStepTimeouts timeouts;
		// const timeouts = this._sequenceStepTimeouts;
		// const callback = cb;
		this._getSequenceStepTimeouts( (timeouts) => {

			const enables = this._sequenceStepEnables;

			const StartOverhead       = 1320; // note that this is different than the value in get_ 
			const EndOverhead         = 960;
			const MsrcOverhead        = 660;
			const TccOverhead         = 590;
			const DssOverhead         = 690;
			const PreRangeOverhead    = 660;
			const FinalRangeOverhead  = 550;
			const MinTimingBudget 	  = 20000;

			// if ( budget_us < MinTimingBudget ) { return false; }

			let used_budget_us = StartOverhead + EndOverhead;

			// getSequenceStepEnables(&enables);
			// getSequenceStepTimeouts(&enables, &timeouts);

			if (enables.tcc) {
				used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
			}

			if (enables.dss) {
				used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
			}
			else if (enables.msrc) {
				used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
			}

			if (enables.pre_range) {
				used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
			}
			
			if (enables.final_range) {
				used_budget_us += FinalRangeOverhead;

				// "Note that the final range timeout is determined by the timing
				// budget and the sum of all other timeouts within the sequence.
				// If there is no room for the final range timeout, then an error
				// will be set. Otherwise the remaining time will be applied to
				// the final range."

				// if (used_budget_us > budget_us) {
				// 	// "Requested timeout too big."
				// 	console.log("FAIL");
				// 	console.log(used_budget_us, budget_us)
				// 	return false;
				// }

				let final_range_timeout_us = budget_us - used_budget_us;

				// set_sequence_step_timeout() begin
				// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

				// "For the final range timeout, the pre-range timeout
				//  must be added. To do this both final and pre-range
				//  timeouts must be expressed in macro periods MClks
				//  because they have different vcsel periods."

				let final_range_timeout_mclks = this.timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

				if (enables.pre_range) {
					final_range_timeout_mclks += timeouts.pre_range_mclks;
				}

				// writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, this.encodeTimeout(final_range_timeout_mclks));
				console.log(final_range_timeout_mclks);

				this._writeRegisters( REGISTRY.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, this.encodeTimeout(final_range_timeout_mclks) );

				// set_sequence_step_timeout() end

				this._measurement_timing_budget_us = budget_us; // store for internal reuse
			}
      // callback()
			console.log(this._measurement_timing_budget_us)
		});
		return true;

	}

	/*
	------------------------------------------
	| getMeasurementTimingBudget:void (-)
	| Get the measurement timing budget in microseconds
	| based on VL53L0X_get_measurement_timing_budget_micro_seconds()
	| in us
	------------------------------------------ */
	getMeasurementTimingBudget( callback ){
		const enables = this._sequenceStepEnables;
		// const timeouts = this._sequenceStepTimeouts;

		this._getSequenceStepTimeouts( (timeouts) => {
			const StartOverhead      = 1910; // note that this is different than the value in set_
			const EndOverhead        = 960;
			const MsrcOverhead       = 660;
			const TccOverhead        = 590;
			const DssOverhead        = 690;
			const PreRangeOverhead   = 660;
			const FinalRangeOverhead = 550;

			// "Start and end overhead times always present"
			let budget_us = StartOverhead + EndOverhead;

			if (enables.tcc) {
				budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
			}

			if (enables.dss) {
				budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
			}
			else if (enables.msrc) {
				budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
			}

			if (enables.pre_range) {
				budget_us += (timeouts.pre_range_us + PreRangeOverhead);
			}

			if (enables.final_range) {
				budget_us += (timeouts.final_range_us + FinalRangeOverhead);
			}

			this._measurement_timing_budget_us = budget_us; // store for internal reuse

			callback( budget_us );
			return budget_us;
		});
	}

	/*
	------------------------------------------
	| setVcselPulsePeriod:bool (-)
	|
	| Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
	| given period type (pre-range or final range) to the given value in PCLKs.
	| Longer periods seem to increase the potential range of the sensor.
	| Valid values are (even numbers only):
	|  pre:  12 to 18 (initialized default: 14)
	|  final: 8 to 14 (initialized default: 10)
	| based on VL53L0X_set_vcsel_pulse_period()
	------------------------------------------ */
	setVcselPulsePeriod(vcselPeriodType, period_pclks, callback){
		let vcsel_period_reg = this._encodeVcselPeriod(period_pclks);

		const enables = this._sequenceStepEnables;
		
		this._getSequenceStepTimeouts( ( timeouts ) => {

			// "Apply specific settings for the requested clock period"
			// "Re-calculate and apply timeouts, in macro periods"

			// "When the VCSEL period for the pre or final range is changed,
			// the corresponding timeout must be read from the device using
			// the current VCSEL period, then the new VCSEL period can be
			// applied. The timeout then must be written back to the device
			// using the new VCSEL period.
			//
			// For the MSRC timeout, the same applies - this timeout being
			// dependant on the pre-range vcsel period."

			if (vcselPeriodType == "VcselPeriodPreRange") {
				// "Set phase check limits"
				switch (period_pclks) {
					case 12:
						this._writeRegisters(REGISTRY.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
					break;
					case 14:
						this._writeRegisters(REGISTRY.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
					break;
					case 16:
						this._writeRegisters(REGISTRY.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
					break;
					case 18:
						this._writeRegisters(REGISTRY.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
					break;
					default:
					// invalid period
						return false;
				}

				this._writeRegisters(REGISTRY.PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

				// apply new VCSEL period
				this._writeRegisters(REGISTRY.PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
				// update timeouts

				// set_sequence_step_timeout() begin
				// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

				let new_pre_range_timeout_mclks = this.timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

				this._writeRegisters(REGISTRY.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, this.encodeTimeout(new_pre_range_timeout_mclks));

				// set_sequence_step_timeout() end

				// set_sequence_step_timeout() begin
				// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

				let new_msrc_timeout_mclks = this.timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

				this._writeRegisters(REGISTRY.MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

				// set_sequence_step_timeout() end
			} else if (vcselPeriodType == "VcselPeriodFinalRange") {

				switch (period_pclks){
					case 8:
						this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
						this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
						this._writeRegisters(REGISTRY.GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
						this._writeRegisters(REGISTRY.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
						this._writeRegisters(0xFF, 0x01);
						this._writeRegisters(REGISTRY.ALGO_PHASECAL_LIM, 0x30);
						this._writeRegisters(0xFF, 0x00);
					break;

					case 10:
						this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
						this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
						this._writeRegisters(REGISTRY.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
						this._writeRegisters(REGISTRY.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
						this._writeRegisters(0xFF, 0x01);
						this._writeRegisters(REGISTRY.ALGO_PHASECAL_LIM, 0x20);
						this._writeRegisters(0xFF, 0x00);
					break;

					case 12:
						this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
						this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
						this._writeRegisters(REGISTRY.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
						this._writeRegisters(REGISTRY.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
						this._writeRegisters(0xFF, 0x01);
						this._writeRegisters(REGISTRY.ALGO_PHASECAL_LIM, 0x20);
						this._writeRegisters(0xFF, 0x00);
					break;

					case 14:
						this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
						this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
						this._writeRegisters(REGISTRY.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
						this._writeRegisters(REGISTRY.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
						this._writeRegisters(0xFF, 0x01);
						this._writeRegisters(REGISTRY.ALGO_PHASECAL_LIM, 0x20);
						this._writeRegisters(0xFF, 0x00);
					break;

					default:
					// invalid period
					return false;
				}

				// apply new VCSEL period
				this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

				// update timeouts

				// set_sequence_step_timeout() begin
				// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

				// "For the final range timeout, the pre-range timeout
				//  must be added. To do this both final and pre-range
				//  timeouts must be expressed in macro periods MClks
				//  because they have different vcsel periods."

				let new_final_range_timeout_mclks = this.timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

				if (enables.pre_range) {
					new_final_range_timeout_mclks += timeouts.pre_range_mclks;
				}

				this._writeRegisters(REGISTRY.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, this.encodeTimeout(new_final_range_timeout_mclks));

				// set_sequence_step_timeout end
			} else {
		    // invalid type
		    return false;
		  }

			// "Finally, the timing budget must be re-applied"

			this.setMeasurementTimingBudget(this._measurement_timing_budget_us);

			// "Perform the phase calibration. This is needed after changing on vcsel period."
			// VL53L0X_perform_phase_calibration() begin

			this._readRegisters(REGISTRY.SYSTEM_SEQUENCE_CONFIG, 16, (err, data) => {
				let sequence_config = data.readInt8();

				this._writeRegisters(REGISTRY.SYSTEM_SEQUENCE_CONFIG, 0x02, () => {
					this.performSingleRefCalibration(0x0);

					this._writeRegisters(REGISTRY.SYSTEM_SEQUENCE_CONFIG, sequence_config, (data) => {
						if( callback ){
							callback(data)
						}
					});

					// VL53L0X_perform_phase_calibration() end
					return true;
				});
			});

		});
	}

	/*
	------------------------------------------
	| getVcselPulsePeriod:void (-)
	| Get the VCSEL pulse period in PCLKs for the given period type.
	| based on VL53L0X_get_vcsel_pulse_period()
	| ## TODO: Come Back
	------------------------------------------ */
	getVcselPulsePeriod(vcselPeriodType){
		if (type == VcselPeriodPreRange) {
			return this._decodeVcselPeriod(this._readRegisters(REGISTRY.PRE_RANGE_CONFIG_VCSEL_PERIOD));
		}
		else if (type == VcselPeriodFinalRange) {
			return this._decodeVcselPeriod(this._readRegisters(REGISTRY.FINAL_RANGE_CONFIG_VCSEL_PERIOD));
		}
		else { return 255; }
	}

	/*
	------------------------------------------
	| startContinuous:void (-)
	| Start continuous ranging measurements. If period_ms (optional) is 0 or not
	| given, continuous back-to-back mode is used (the sensor takes measurements as
	| often as possible); otherwise, continuous timed mode is used, with the given
	| inter-measurement period in milliseconds determining how often the sensor
	| takes a measurement.
	| based on VL53L0X_StartMeasurement()
	------------------------------------------ */
	startContinuous(period_ms){
		this._writeRegisters(0x80, 0x01);
		this._writeRegisters(0xFF, 0x01);
		this._writeRegisters(0x00, 0x00);
		this._writeRegisters(0x91, this._stop_variable);
		this._writeRegisters(0x00, 0x01);
		this._writeRegisters(0xFF, 0x00);
		this._writeRegisters(0x80, 0x00);
		if (period_ms != 0) {
			// continuous timed mode
			// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

			this._readRegisters(REGISTRY.OSC_CALIBRATE_VAL, 16, (err, data) => {
				let osc_calibrate_val = data;
				if (osc_calibrate_val != 0) {
					period_ms *= osc_calibrate_val;

					this._writeRegisters(REGISTRY.SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

					// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
					this._writeRegisters(REGISTRY.SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
				} else {
					// continuous back-to-back mode
					this._writeRegisters(REGISTRY.SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
				}
			});
		}
	}

	/*
	------------------------------------------
	| stopContinuous:void (-)
	|
	| Stop continuous measurements
	| based on VL53L0X_StopMeasurement()
	------------------------------------------ */
	stopContinuous(){
		this._writeRegisters(REGISTRY.SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

		this._writeRegisters(0xFF, 0x01);
		this._writeRegisters(0x00, 0x00);
		this._writeRegisters(0x91, 0x00);
		this._writeRegisters(0x00, 0x01);
		this._writeRegisters(0xFF, 0x00);
	}

	/*
	------------------------------------------
	| readRangeContinuousMillimeters:void (-)
	| Returns a range reading in millimeters when continuous mode is active
	| (readRangeSingleMillimeters() also calls this function after starting a
	| single-shot range measurement)
	| ## TODO: Come back
	------------------------------------------ */
	readRangeContinuousMillimeters(){
		this._startTimeout();

		while ((this._readRegisters(REGISTRY.RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
			if (checkTimeoutExpired()) {
				this._did_timeout = true;
				return 65535;
			}
		}

		// assumptions: Linearity Corrective Gain is 1000 (default);
		// fractional ranging is not enabled
		let range = this._readRegisters(REGISTRY.RESULT_RANGE_STATUS + 10);

		this._writeRegisters(REGISTRY.SYSTEM_INTERRUPT_CLEAR, 0x01);

		return range;
	}

	/*
	------------------------------------------
	| readRangeSingleMillimeters:void (-)
	| uint16_t
	------------------------------------------ */
	readRangeSingleMillimeters(){

	}

	/*
	------------------------------------------
	| timeoutOccurred:bool (-)
	------------------------------------------ */
	timeoutOccurred(){
		let tmp = this._did_timeout;
		this._did_timeout = false;
		return tmp;
	}

	/*
	------------------------------------------
	| getSpadInfo:bool (-)
	------------------------------------------ */
	getSpadInfo( count, type_is_aperture ){

	}

	/*
	------------------------------------------
	| performSingleRefCalibration:void (-)
	| based on VL53L0X_perform_single_ref_calibration()
	------------------------------------------ */
	performSingleRefCalibration( vhv_init_byte ){
		this._writeRegisters(REGISTRY.SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

		// // startTimeout();
		// while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
		// {
		//   if (checkTimeoutExpired()) { return false; }
		// }

		this._writeRegisters(REGISTRY.SYSTEM_INTERRUPT_CLEAR, 0x01);
		this._writeRegisters(REGISTRY.SYSRANGE_START, 0x00);

		return true;
	}

	/*
	------------------------------------------
	| decodeTimeout:uint16_t (-)
	| Decode sequence step timeout in MCLKs from register value
	| based on VL53L0X_decode_timeout()
	| Note: the original function returned a uint32_t, but the return value is
	| always stored in a uint16_t.
	------------------------------------------ */
	decodeTimeout( value ){
		return ((value & 0x00FF) << ((value & 0xFF00) >> 8)) + 1;
	}

	/*
	------------------------------------------
	| encodeTimeout:uint16_t (-)
	| Encode sequence step timeout register value from timeout in MCLKs
	| based on VL53L0X_encode_timeout()
	| Note: the original function took a uint16_t, but the argument passed to it
	| is always a uint16_t.
	------------------------------------------ */
	encodeTimeout( timeout_mclks ){
		// format: "(LSByte * 2^MSByte) + 1"

		let ls_byte = 0;
		let ms_byte = 0;

		if (timeout_mclks > 0) {
			ls_byte = timeout_mclks - 1;

			while ((ls_byte & 0xFFFFFF00) > 0) {
				ls_byte >>= 1;
				ms_byte++;
			}

			return (ms_byte << 8) | (ls_byte & 0xFF);
		}
		else { return 0; }
	}

	/*
	------------------------------------------
	| timeoutMclksToMicroseconds:uint32_t (-)
	| Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
	------------------------------------------ */
	timeoutMclksToMicroseconds( timeout_period_mclks, vcsel_period_pclks ){
		let macro_period_ns = this._calcMacroPeriod(vcsel_period_pclks);
		return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
	}

	/*
	------------------------------------------
	| timeoutMicrosecondsToMclks:uint32_t (-)
	| Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
	| based on VL53L0X_calc_timeout_mclks()
	------------------------------------------ */
	timeoutMicrosecondsToMclks( timeout_period_us, vcsel_period_pclks ){
		let macro_period_ns = this._calcMacroPeriod(vcsel_period_pclks);
	  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
	}

	/*
	------------------------------------------
	| _readRegisters:void (-)
	|
	| Read the registers via I2C.
	------------------------------------------ */
	_readRegisters(addressToRead, bytesToRead, callback){
		this.queue.place( () => {
			this.i2c.transfer( new Buffer([addressToRead]), bytesToRead, (err, data) => {
				this.queue.next();
				if( callback ){
					callback(err, data);
				}
			})
		});
	}

	/*
	------------------------------------------
	| _writeRegisters:void (-)
	|
	| Write to the register via I2C.
	------------------------------------------ */
	_writeRegisters(addressToRead, dataToWrite, callback){
		this.queue.place( () => {
			this.i2c.send( new Buffer([addressToRead, dataToWrite]), (err, data) => {
				this.queue.next();
				if( callback ){
					callback(err, data);
				}
			})
		});
	}

}

module.exports = VL53LOX;
