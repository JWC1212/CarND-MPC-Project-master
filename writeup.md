**MPC Control Project**

The goals / steps of this project are the following:

* In this project a MPC controller is implemented in C++ to maneuver the vehicle around the track.
* Cross track error (CTE) and orientation error (epsi) are provided in order to compute the appropriate actuator values: steering angle and throttle.

---

#  The Model.
## The car's state is described as a 6-element vector in this model: 
      position(x), position(y), orientation of car(psi), speed(v), cross track error(cte) and orientation error(epsi).
## MPC will generate an output actuator vector consisting of steering angle(delta) and throttle(a) both in closed interval [-1, 1].
## The state is updated by following equations:
   |	x1    =    x0 + v0 * cos(psi0) * dt			|
   |	y1    =    y0 + v0 * sin(psi0) * dt			|
   |	psi1  =  psi0 + v0 * delta0 / Lf * dt			|
   |	v1    =    v0 + a0 * dt					|
   |	cte1  =  cte0 + v0 * sin(epsi0) * dt			|	
   |	epsi1 = epsi0 + v0 * delta0 / Lf * dt			|
 

#  Timestep Length and Elapsed Duration(N & dt)
## Predictive duration T = N * dt is calculated. Longer T will lead to more inaccurate predictive path in case that the speed is high.
## There exists a trade-off between N and dt. If dt is too large the vehicle has experience a big state change and environment variables. Smaller dt will make prediction precisive at each time step so in fact it is a finer resolution. 
## Larger N does not always work. Because Large N leads to large computational time and consequent inaccurate prediction. 
## Finally I used N = 10 and dt = 0.09.

#  Polynomial Fitting and MPC Preprocessing
## The data received from simulation server is in global coordinate system different from the vehicle system in which x is the heading of         the vehicle.
## 3-order polynomial is used to fit the trajectory f(x) = a3 * x * x * x + a2 * x * x  + a1 * x + a0
## Transformation matrix used from global system to vehicle coordinate as below:
	|cos(-psi)	-sin(-psi)|
	|sin(-psi)    	 cos(-psi)|

#  MPC with Latency
## Running a simulation using the vehicle model starting from the current state for the duration of the latency. Receiving steering value and throttle value from simulation then adding them to the state vector. 100ms is taken account into and calculate the new initial state vector.The resulting state from the simulation is the new initial state for MPC.

