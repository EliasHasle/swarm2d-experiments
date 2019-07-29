"use strict";

function PID(dim, optimizer) {
	this.dim = dim || 1;
	this.optimizer = optimizer || new ParameterOptimizer({
		Kp: {val: 0.5, minval: 0, minLR: 0.01},
		Ki: {val: 0.1, minval: 0, minLR: 0.01},
		Kd: {val: 0.1, minval: 0, minLR: 0.01},
		iDecay: {val: 1, minval: 0, maxval: 1, scale: 0.1},
		dSmoothing: {val: 0, minval: 0, maxval: 1, scale: 0.1}
	}, threshold=60);

	this.lastError = new Array(dim).fill(0);
	this.D = new Array(dim).fill(0);
	this.I = new Array(dim).fill(0);
}

PID.prototype = Object.create(Object.prototype);

Object.assign(PID.prototype, {
	constructor: PID,
	reset: function() {
		this.lastError.fill(0);
		this.D.fill(0);
		this.I.fill(0);
	},
	update: function(state, target, dt) {
		let error = target.slice();
		let sumSqError = 0;
		for (let i = 0; i<this.dim; i++) {
			error[i] -= state[i];
			sumSqError += error[i]**2;
		}
		this.optimizer.accumulate(sumSqError);
		
		//Get latest parameter values from optimizer:
		let {Kp, Ki, Kd, iDecay, dSmoothing} = this.optimizer.getParameters();

		let C = this.lastError; //simple reuse of array (for performance, not readability)
		for (let i = 0; i < this.dim; i++) {
			//Slightly smoothed differential:
			this.D[i] = dSmoothing*this.D[i] + (1-dSmoothing)*(error[i]-this.lastError[i])/dt;
			//Slightly decaying integral:
			this.I[i] = iDecay*this.I[i] + error[i]*dt;
			C[i] = Kp*error[i] + Ki*this.I[i] + Kd*this.D[i];
		}	
		this.lastError = error;
		
		return C;
	}
});