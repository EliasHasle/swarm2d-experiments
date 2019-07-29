"use strict";
/*
A collection of basic steering behaviors that can be combined. Not all are great implementations.

Depends on Vector2D_operations.js

TODO: Maybe make the functions that output directions or velocities output accelerations instead, with the help of a few extra parameters and helper functions.
*/

/*Conversions between velocity-aligned (ideal steering) space and world space.*/

//Position conversions:
//ps is local position to convert, pa is global agent position, theta is global agent direction				
function posSteerToWorld(ps, pa, theta) {
	return add(pa, rotate(ps, theta));
}
function posWorldToSteer(pw, pa,va) {
	return rotate(delta(pa,pw), -angle(va));
}

//Velocity conversions
function velSteerToWorld([vForward,theta]) {
	return rotate([vForward,0], theta);
}
function velWorldToSteer(v) {
	return [length(v), -angle(v)];
}

//Acceleration conversions
function accSteerToWorld(aSteer, theta) {
	return rotate(aSteer, theta);
}
function accWorldToSteer(a, v) {
	return rotate(a, -angle(v));
}

//Maximize total acceleration without exceeding component maxes:
function clampSteering(afd,ald, afMax,alMax) {
	let m = Math.min(afMax/afd, alMax/ald);
	return scale([afd,ald], m);
}

/*Acceleration is found (naively) from difference between desired and current velocity.
aMax defaults to 1, which will give an acceleration direction.*/
function velToAcc(v, vd, aMax=1) {
	if (vd===null) return null;
	if (equalVec(v,vd)) return [0,0];
	return scale(normalize(delta(v,vd)), aMax);
}
function dirToAcc(d, v=[0,0], vMax=Math.max(50,length(v)), aMax=1) {
	if (d===null) return null;
	return velToAcc(v, scale(d,vMax), aMax);
}

/*afMax is maximal acceleration in forward direction,
and alMax is maximal lateral acceleration.*/
function accSteer(v, vd, afMax, alMax) {
	let [afd, ald] = accWorldToSteer(velToAcc(v,vd, Math.max(afMax,alMax)), v);
	return clampSteering(afd,ald, afMax,alMax);
}

//Functions that return a desired direction in world space:
//Naive seek:
function seek(a, g) {
	return normalize(delta(a,g));
}

function flee(a, n) {
	return negate(seek(a, n));
}

/*h for "hunter", p for "prey", c is a tuning parameter that estimates the relation between distance and time to impact.
An improvement would be to account for velocities, headings (if applicable) and (presumed) maximal accelerations.
*/
function pursue(h, p,pv, c) {
	let d = length(delta(h,p));
	let T = d*c;
	
	let g = add(p, scale(pv, T));
	
	return seek(h, g);
}

//Will evade, but only if distance is above cutoff. (Hacky?)
function evade(p, h,hv,  c, cutoff=Infinity) {
	if (length(delta(p,h)) > cutoff) return null;
	return negate(pursue(p, h,hv,  c));
}

//Not great...
function follow_path(p,v, vertices,edges, maxR, vdMax,aMax, backward=false, T=0.5) {
	let futurePos = add(p, scale(v,T));
	let near = nearestLinePoint(futurePos, vertices,edges);
	if (near===null) return null;

	if (length(v) > 10 && near.distance <= maxR)
		return null; //no correction needed
	
	if (near.type === "vertex") {
		return dirToAcc(seek(p,add(near.point,[0.1*Math.random(),0.1*Math.random()])), v, vdMax,aMax);
		//return seekAcc(p,v,add(near.point,[0.1*Math.random(),0.1*Math.random()]),aMax); //may be insufficient
	}
	console.log(near);
	if (backward) return dirToAcc(seek(p,near.fromPoint), v, vdMax,aMax);
	return dirToAcc(seek(p,near.toPoint), v, vdMax,aMax);
	//if (backward) return seekAcc(p,v,near.fromPoint,aMax);
	//return seekAcc(p,v,near.toPoint,aMax);
}

//Not great...
function follow_wall(p,v, vertices,edges, offset, aMax, T=Infinity) {
	let cutoff = (length(v)||5)*T;
	let intersection = intersectNearest(p,normalize(v), vertices,edges);
	if (intersection === null || intersection.distance > cutoff) return null;
	let g = add(intersection.point, scale(intersection.normal, offset));
	return seekAcc(p,v,g,aMax);
}

/*Returns an acceleration away from the nearest wall straight ahead, probed by three parallel rays. This is nearly the same as the "containment" behavior defined in Reynolds' paper, when T is set to a reasonably small positive number.*/
function flee_walls(p,v,r, vertices,edges, aMax,T=Infinity) {
	let nearest = null;
	let minDistance = length(v)*T + r;
	let side = normalize(rotate(v, 0.5*Math.PI));
	//Use exactly three parallel probe rays (rays are not always sufficient)
	let dir = normalize(v);
	for (let o = -r; o < 2*r; o+=r) {
		let rayOrigin = add(p, scale(side,o));
		let intersection =
			intersectNearest(
				rayOrigin,
				dir,
				vertices,
				edges);
		if (intersection === null) continue;
		if (intersection.distance <= minDistance) {
			minDistance = intersection.distance;
			nearest = intersection;
		}
	}
	
	if (nearest === null) return null;
	
	//return negate(nearest.normal); //DEBUG (should be wrong)
	return scale(nearest.normal, aMax);
}

//Functions that return an acceleration:

/*Trying to make a seek function that returns an acceleration, 
and more properly minimizes the time to impact.

	First, estimate time to impact from vr and aMax only:
	r-vr*t-0.5*aMax*t**2 = 0
	t = (-vr +-sqrt(vr**2+2*r*aMax))/aMax
	
	Using this t, find the offset caused by the tangential velocity:
	offset = vt*t
	
	Try to compensate for the offset, by applying an acceleration opposing vt:
	offset = vt*t + 0.5*at*t**2 = 0
	Assuming t is the same:
	at = -2*vt/t
	
	But then
	ar = sqrt(aMax**2-(2*vt/t)**2) < aMax
	and
	t = (-vr +-sqrt(vr**2+2*r*ar))/ar
	
	//Below code is a working fixed-point approach (not theoretically proven)
	//Can the equations be solved analytically?
	*/
function seekAcc(pa,va, pg, aMax) {
	let d = delta(pa,pg);
	let r = length(d);
	//Important check to avoid division by zero:
	if (r === 0) {
		return [0,0];
	}	
	let dir = scale(d, 1/r);
	
	//RT coordinates will be useful:
	let vr = dot(va,dir);
	let vt = dot(va, rotate(dir, 0.5*Math.PI));
	let theta = angle(dir);
	
	//Trusting to find a fixed point by iteration
	//(this decision must be verified mathematically):
	let ar = aMax, at=0;
	let dat = Infinity;
	while (ar !==0 && Math.abs(dat) > 0.0001*aMax) {
		//The kernel must be non-negative:
		let srt = (vr**2+2*r*ar)**0.5;
		let t1 = (-vr-srt)/ar;
		let t2 = (-vr+srt)/ar;
		//Simplification (t <= 0 is now impossible):
		let t = t1 > 0 ? t1 : t2;
		//Even simpler, but not correct:
		//let t = vr < 0 ? t1 : t2;
		let atn = Math.max(-aMax,Math.min(aMax,-2*vt/t));
		dat = atn-at;
		at = atn;
		ar = (aMax**2 -(at**2))**0.5; //how about negative ar? Never good, I suppose.
	}
	
	return rotate([ar,at], theta);
}

//Pursue based on better seek
function pursueAcc(pa,va, pt,vt, aMax) {
	return seekAcc(pa, sub(va,vt), pt, aMax);
}

/*Slightly different from the basic definition with a fixed critical distance, this implementation assumes a maximal acceleration instead (possibly already discounted for priority), and returns an acceleration.

I would like to generalize this further, to make a "docking" behavior that approaches a moving target, and aligns its velocity with the target during "arrival". An attempt is done below at making docking behavior using arrival behavior.
*/
function arrival(p,v, g, aMax) {
	let d = sub(g,p); //vector pointing TO goal
	let r = length(d);
	let gdir = scale(d, 1/r); //normalize
	let speed = length(v);
	
	if (r === 0) {
		//console.log("arrival: r == 0");
		if (speed > 0) return scale(v, -aMax/speed);
		return [0,0];
	}
	
	let sa = seekAcc(p,v,g,aMax);
	let ar = dot(sa,gdir);
	let at = dot(sa,rotate(gdir,0.5*Math.PI));
	let theta = angle(gdir);

	if (speed === 0) {
		if (d > 0) return sa;
		else return [0,0];
	}

	let vdir = scale(v, 1/speed); //normalize;
	
	//Inward radial speed
	let vr = dot(v, gdir);
	if (vr < 0) {
		//console.log("Moving away. Seek!");
		return sa;
		//return dirToAcc(seek(p,g), v, Math.max(2*speed,20), aMax);
	}

	//NOTE: Replace aMax with planned ar to get a better estimate,
	//because the whole of aMax is not free for braking. This modification gives a smaller overshoot.
	//d(t) = r -vr*t +0.5*aMax*t**2 = 0
	//Solve for t, to get a conservative estimate for time to impact:
	//t = (vr +- sqrt(vr**2 - 2*aMax*r))/aMax
	let kernel = vr**2 - 2*ar/*aMax*/*r;
	let vrd;
	if (kernel < 0) {
		//No impact if braking aMax. Can then simply seek toward goal:
		//console.log("No impact if braking. Seek!");
		return sa;
		//return dirToAcc(seek(p,g), v, Math.max(2*speed,20), aMax);
	} else {
		//console.log("Brake!");
		return rotate([-ar, at], theta);
		//return seekAcc(g,v,p,aMax); //swapped g,p
	}
}

/*Very experimental, maybe not so good. The idea is to aim for arrival (zero speed) exactly so far in front of the target that the velocity when at the same distance as the target is equal to the velocity of the target. I at least expect this to work for targets that move with constant velocity. I also expect that increasing aMax will improve docking performance, even for nonlinear target motion. As aMax grows, the offset shrinks toward zero.

Recapitulate 1D constant acceleration equation:
(s-s0) = v0*t + 0.5*a0*t**2 = vg*t - 0.5*aMax*t**2
//Time to stop:
v = v0 + 0.5*a0*t = vg -aMax*t = 0 ==> t = vg/aMax
//Use in above equation:
(s-s0) = vg**2/aMax - 0.5*vg**2/aMax = 0.5*vg**2/aMax
*/
function docking(p,vp, g,vg, aMax) {
	return arrival(p,sub(vp,vg), g, aMax);
	//return arrival(p,vp, add(g, scale(vg,0.5*length(vg)/aMax)), aMax);
}

//ps is an array of all positions to consider, where pIndex will be ignored. Returns an acceleration
function separation(p,pIndex, ps, aMax, cutoff=Infinity) {
	if (ps.length <= 1) return null;
	//if (Math.random()<0.01) console.log("DEBUG: ps[0] = ", ps[0]);
	let cutoff2 = cutoff**2;
	let a = null;
	for (let i = 0; i < ps.length; i++) {
		if (i===pIndex) continue;
		
		let d = delta(ps[i], p);
		let d2 = lengthSq(d);
		if (d2 > cutoff2) continue;
		
		a = add(a || [0,0], scale(d, aMax/(d2+1))); //+1 for numerical stability
	}
	if (a===null) return null;
	return scale(clampLength(a, 1), aMax);
}

//Returns an inward acceleration, as in the original boids model
function cohesion(p,pIndex, ps, aMax, cutoff=Infinity) {
	if (ps.length <= 1) return null;
	let cutoff2 = cutoff**2;
	let center = null;
	let n = 0;
	for (let i = 0; i < ps.length; i++) {
		if (i===pIndex) continue;
		let d = delta(ps[i], p);
		let d2 = lengthSq(d);
		if (d2 > cutoff2) continue;
		
		center = add(center || [0,0], ps[i]);
		n++;
	}
	if (center === null) return null;
	center = scale(center, 1/n);
	return scale(normalize(delta(p, center)), aMax);
}

/*Similar to cohesion, but uses a radial basis weighting (rbFunc) of the positions, instead of a simple average*/
function cohesionRB(p,pIndex, ps, rbFunc, aMax, cutoff=Infinity) {
	if (ps.length <= 1) return null;
	let cutoff2 = cutoff**2;
	let center = [0,0];
	let sumWeights = 0;
	for (let i = 0; i < ps.length; i++) {
		if (i===pIndex) continue;
		
		let r = delta(ps[i], p);
		let r2 = lengthSq(r);
		if (r2 > cutoff2) continue;
		
		let w = rbFunc(r);
		sumWeights += w;
		center = add(center, scale(ps[i], w));
	}
	center = scale(center, 1/sumWeights);
	return scale(normalize(delta(p, center)), aMax);
}

//Returns a velocity
function alignment(p,ownIndex, ps,vs, cutoff=Infinity) {
	if (ps.length <= 1) return null;
	let cutoff2 = cutoff**2;
	let vAvg = null;
	let n = 0;
	for (let i = 0; i < ps.length; i++) {
		if (i===ownIndex) continue;
		
		let d = delta(ps[i], p);
		let d2 = lengthSq(d);
		if (d2 > cutoff2) continue;
		
		vAvg = add(vAvg || [0,0], vs[i]);
		n++;
	}
	if (n===0) return null;
	vAvg = scale(vAvg, 1/n);
	return vAvg;
	//return normalize(vAvg); //To get a direction instead
}

//radial basis version of alignment too
function alignmentRB(p,pIndex, ps,vs, rbFunc, cutoff=Infinity) {
	if (ps.length <= 1) return null;
	let cutoff2 = cutoff**2;
	let vAvg = [0,0];
	let sumWeights = 0;
	for (let i = 0; i < ps.length; i++) {
		if (i===pIndex) continue;
		
		let r = delta(ps[i], p);
		let r2 = lengthSq(r);
		if (r2 > cutoff2) continue;
		
		let w = rbFunc(r);
		sumWeights += w;
		vAvg = add(vAvg, scale(vs[i], w));
	}
	vAvg = scale(vAvg, 1/sumWeights);
	return vAvg;
}

/*Simple wander implementation (not like in the paper, based on sampling a sphere surface in front of the agent).
The implementation does not work too well. It is almost equivalent to sampling a random acceleration at each time step, so most of the time the sequence of accelerations seems to almost null itself out (as predicted by the law of large numbers).
Optional W and H are width and height of the space (assuming 0,0 is in a corner)*/
function wander([x,y],[vx,vy], desiredSpeed, aMax, W,H) {
	//Keep the wandering agent within the screen:
	let [ax,ay] = [0,0];
	if (!isNaN(W)) {
		if (x < 0 && vx < 0) ax = aMax;
		else if (x > W && vx > 0) ax = -aMax;
	}
	if (!isNaN(H)) {
		if (y < 0 && vy < 0) ay = aMax;
		else if (y > H && vy > 0) ay = -aMax;
	}
	if (ax || ay) return clampLength([ax,ay], aMax);
	
	let [vf,theta] = velWorldToSteer([vx,vy]);
	let vsd = vf*(2*random()-1);
	let vd = scale(normalize(rotate([vf,vsd], theta)), desiredSpeed);
	
	([ax,ay] = velToAcc([vx,vy], vd, aMax));
	
	return [ax,ay];
}

/*Avoid round, static obstacles (could be bounding circles of complex geometries). Similar to the obstacle avoidance in Reynolds' paper, but I have used my own reasoning for the strength of the controls.
p,v,r are world position, world velocity and radius of agent.
pws,rs are world positions and radii of obstacles.
afMax,alMax are limits of steering accelerations. If combining policies, afMax and alMax are presumably already discounted.
Returns a global acceleration.*/
function avoid_obstacles(p,v,r, pws,rs, afMax,alMax) {
	let speed = length(v);
	let cutoff = speed**2/afMax;
	let minDistance = cutoff;
	let afd = 0;
	let ald = 0;
	for (let i = 0; i < pws.length; i++) {
		let pw = pws[i];
		let [oF,oL] = posWorldToSteer(pw, p,v);
		//If obstacle is within dangerous distance:
		if (oF>-r && oF<minDistance+r) {
			let oR = rs[i]; //obstacle radius
			//Then if obstacle intersects swept agent
			if (Math.abs(oL)<oR+r) {
				minDistance = oF;
				let minCollDist = oF-oR-r;
				afd = -(speed**2)/minCollDist;
				ald = afd*Math.sign(oL)*(oR+r)/minCollDist;
			}		
		}
	}

	let aSteer = clampSteering(afd,ald, afMax,alMax);
	return accSteerToWorld(aSteer, angle(v));
}

/*Combine several behaviors into one behavior by weighting them.
v is the agent velocity. dirs,vels,accs are arrays of directions, velocities and accelerations. dws,vws,aws are weights. vdMax is the maximum allowed desired velocity. aMax is the maximal acceleration.*/
function combine({v, dirs,dws, vels,vws, accs,aws, vdMax,aMax}) {
	vdMax = vdMax || 1;
	aMax = aMax || 1;
	//Use only copies of the arrays that might be modified:
	if (vels) vels = vels.slice();
	if (vws) vws = vws.slice();
	if (accs) accs = accs.slice();
	if (aws) aws = aws.slice();
	
	if (dirs && dws) {
		let velAvgDir = scale(averageVector(dirs,dws) || [0,0], vdMax);
		let WD = 0;
		for (let i = 0; i < dws.length; i++) { 
			if (dirs[i] !== null) WD += dws[i] || 0;
		}
		if (!vels) {
			vels = [];
			vws = [];
		}
		vels.push(velAvgDir);
		vws.push(WD);
	}
	
	if (vels && vws) {
		let avgVel = averageVector(vels,vws) || [0,0];
		let WV = 0;
		for (let i = 0; i < vws.length; i++) { 
			if (vels[i] !== null) WV += vws[i] || 0;
		}
		if (!accs) {
			accs = [];
			aws = [];
		}
		accs.push(velToAcc(v, avgVel, aMax));
		aws.push(WV);
	}
	
	let a = averageVector(accs,aws);
	if (a===null) return null;
	return clampLength(a, aMax);
}

//Return first valid behavior (does not take care of conversions automatically)
function prioritize(behaviors) {
	for (b of behaviors) {
		if (b !== null) return b;
	}
	return null;
}

/*Aim to arrive just behind the leader, while avoiding to get in the leader's way.
The description says to aim for arriving behind the leader,
and conditionally evade laterally wrt. the leader's direction.
This formulation is a bit different. And does not work very well...*/
//Returns an acceleration
function follow_leader(p,pv,pIndex, l,lv, ps, aMax, lag=20) {
	let eSpeed = length(pv)+length(lv);
	
	let e = evade(p, l,lv, 1/eSpeed, Math.max(lag,length(lv)));
	if (e !== null) return velToAcc(pv, scale(e,eSpeed), aMax);
	
	let a = docking(p,pv, sub(l,scale(normalize(lv),lag)), lv, aMax);//arrival(p,pv, sub(l, scale(normalize(lv), lag)), aMax);
	let s = separation(p,pIndex, ps, aMax, 0.5*lag);
	
	return combine({v: pv, accs: [a, s], aws: [0.01, 0.99],aMax});
}

//Testing this for quadratic drag:
function calculateDrag(v, c) {
	return scale(v, -c*length(v));
}