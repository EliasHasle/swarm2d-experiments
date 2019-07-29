"use strict";

//Constructs an optimizer object that uses simulated annealing
//to test and evaluate parameter values.
/*Takes a name; a dictionary of parameters to optimize, where keys are parameter names, and each entry holds an initial value and optional parameters: min value, max value, learning rate scale, minimal learning rate; and a dictionary of optimizer parameters: probWorse is the initial probability of acceptign wrose parameters, minimize is a boolean (if false, maximize), threshold is the initial number of received samples to require before evaluating performance, and pwDecay is how fast the base probWorse decays.*/
function ParameterOptimizer(name, dict, {probWorse, minimize, threshold, pwDecay}={probWorse: 0.6, minimize: true, threshold: 60, pwDecay: 0.98}) {
	this.name = name;

	this.params = {};
	this.baseParams = {};
	for (let key in dict) {
		this.params[key] = {};
		Object.assign(this.params[key], dict[key]);
		this.baseParams[key] = {};
		Object.assign(this.baseParams[key], dict[key]);
		//Deep-copy matrix:
		if (dict[key].type === "matrix") {
			let nm = dict[key].val.length;
			this.params[key].val = dict[key].val.slice();
			this.baseParams[key] = dict[key].val.slice();
		}
	}
	
	this.probWorse = probWorse;
	this.pwDecay = pwDecay;
	this.learningRate = 0.2*probWorse;
	this.minimize = minimize; //false to maximize performance measure
	
	//number of samples to collect before an optimization iteration.
	this.threshold = threshold;
	
	this.sampleCount = 0;
	this.performance = 0;
	
	this.basePerfRate = minimize ? Infinity : -Infinity;
	this.useBase = false;
}
ParameterOptimizer.prototype = Object.create(Object.prototype);
Object.assign(ParameterOptimizer.prototype, {
	constructor: ParameterOptimizer,
	getParameters: function() {
		let params = {};
		for (let param in this.params) {
			//Deep-copy matrix, to be safe
			if (this.params[param].type === "matrix") {
				params[param] = this.params[param].val.slice();
			} else {
				params[param] = this.params[param].val;
			}
			
		}
		return params;
	},
	accumulate: function(deltaperf) {
		this.performance += deltaperf;
		this.sampleCount++;
		if (this.sampleCount === this.threshold && this.performance !== 0) {
			this.evaluate();
		} else if (this.sampleCount >= this.threshold) {
			if (this.performance!==0) {
				this.evaluate();
				//adjust threshold "halfway" back
				if (this.sampeCount>this.threshold) {
					this.threshold /= 2**0.5;
					console.info("Optimizer +"+this.name+": Reducing threshold.");
				}
			} else {
				//adjust threshold up
				this.threshold *= 2;
				console.info("Optimizer +"+this.name+": Increasing threshold.");
			}
		}
	},
	evaluate: function() {
		let perfRate = this.performance/this.sampleCount;

		//DEBUG
		/*if (this.name==="Swarm velocity PID") {
			console.info("Evaluating Swarm velocity PID performance. PR=",perfRate, " samples=",this.sampleCount, " threshold=",this.threshold);
		}*/
		this.performance = 0;
		this.sampleCount = 0;
		
		if (this.useBase) {
			this.bestPerf = perfRate;
			this.useBase = false;
			return;
		}

		let makeMove = false;
		
		if ((this.minimize && perfRate<=this.basePerfRate) 
		|| (!this.minimize && perfRate>=this.basePerfRate)) {
			makeMove = true;
		} else {
			let p = this.probWorse**(1+Math.abs((perfRate-this.basePerfRate)/this.basePerfRate));
			if (Math.random() < p) {
				makeMove = true;
				console.info("Optimizer %s Decides to make worsening move from probability p=%.2f", this.name, p);
			}
		}
		if (makeMove) {
			this.updateParameters(perfRate);
		}
		
		this.useBase = true; //use (verify) base parameters every other turn.
	},
	updateParameters: function(perfRate) {
			console.groupCollapsed("Optimizer %s:", this.name);
			for (let param in this.params) {
				console.info("\t"+param+"="+this.params[param].val);
			}
			console.info("\tWith PW=%.3f, PR=%.2f", this.probWorse, perfRate);
			console.groupEnd();
			
			this.probWorse = this.pwDecay*this.probWorse;
			
			for (let param in this.params) {
				this.updateParameter(param, this.baseParams, this.params);
			}
			this.basePerfRate = perfRate;
	},
	updateParameter: function(key, basep, destp) {
		let par = destp[key];
		if (par.type === "matrix") {
			for (let i = 0; i < basep[key].length; i++) {
				basep[key][i] = par.val[i];
			}
		} else {
			basep[key] = par.val;
		}
		let base = basep[key];
		let mi = par.minval;
		if (typeof mi === "undefined") mi = -Infinity;
		let ma = par.maxval;
		if (typeof ma === "undefined") ma = Infinity;
		let lr = Math.max(par.minLR||0, this.probWorse*(par.scale||1));
		if (par.type === "matrix") {
			for (let i = 0; i < base.length; i++) {
				let rand = (2*Math.random()-1);
				par.val[i] = Math.min(Math.max(base[i]+rand*lr, mi), ma);
				if (isNaN(par.val[i])) {
					console.error("Optimizer "+this.name+": NaN parameter generated!");
				}
			}
		} else {
			let rand = (2*Math.random()-1);
				//Set new parameter value for testing
				par.val = Math.min(Math.max(base+rand*lr, mi), ma);
			if (isNaN(par.val)) {
				console.error("Optimizer "+this.name+": NaN parameter generated!");
			}
		}
	}
});