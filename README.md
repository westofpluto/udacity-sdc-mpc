# Udacity Self Driving Car MPC Project

This repo contains my solution to the Udacity SDC Nanodegree project for Model Predictive Control. The objective was to develop a MPC for acceleration and steering angle to drive autonomously around a simuated track using the Udacity simulator. The MPC solution involves the use of desired waypoint along the road ahead, and at each time step the MPC systems solves an optimization problem to determine the history of control actions that if applied would minimize a cost function while simultaneously satisfying a number of constrainst, specificall teh initial state constraints, the equations of motion constraints, and lowerupper bounds on controller actions. Closed-loop control is achieved by using only the initial set of control actions from the optimization and then repeating this calculation many times per second, using the current state as the initial state for calculations. Actually, since the solution must be robust to 100ms latency, I handle latency by using the current state predicted ahead in time by 100ms as the initial state for the optimization.

The paragraphs below discuss all the points identified the project rubric and provide more detail on my solution to the problem.

## Rubric Points
* Compilation: Code must compile without errors with cmake and make.

The code makes use of the IPOPT and CPPAD libraries. Both of these libraries must be installed to compile and use the program. I had not trouble installing CPPAD, however IPOPT posed more of a challenge. If running this on a Mac, I suggest getting the IPOPT source code repo and building from source. However, assuming that IPOPT and CPPAD are installed, this project should build with no problems.

* The Model: Student describes their model in detail. This includes the state, actuators and update equations.

The MPC system uses an optimal control formulation that optimizes a cost function subject to a number of equations that represent the vehicle initial state and equations of motion. In this problem we used a set of kinemtic equations with 6 states: the vehicle X and Y position, the orientation angle psi, the speed v, cross-track error and psi error. These equations of motion were given in the Vehicle Models lesson, but for this problem had to be slightly modified since the steering angle delta is oriented in the opoosive direction in the simulator as it is in the original equations.

![Alt text](images/equations.png?raw=true "Equations of Motion")

The controls used are the steering angle delat and the throttle represented by the car acceleration a.

At each time step the MPC process begins by transforming the equations of motion into a new frame of reference, the car frame, which is centered at the vehicles current (known) position and oriented along the vehicles known orientation angle. The initial state in the car frame of reference is (x=y=psi=0, v=current speed, and cte and epsi calculated as follows: cte is calculated as f(0) where f(x) is a 3rd order polynomial fit of the desired x,y waypoints on the road ahead. The waypoints are provided in the world coordinate system by the simulator, and the MPC system transforms these into car coordinates. Since cte is defined as desired y - actual y at the current x, and since x=y=0, we get cte = f(0). The value for epsi is similarly found as espi = 0 - psi desired, weher psi desired is the angle whose tangent is the first derivative of f(x) at x=0. The tangent of a 3rd order polynomial c0 + x * c1 + x * x * c2 + x * x * x * c3 is simply c1.

The optimization library IPOPT is used in realtime to find the history of controls a(t) and delta(t) (t=0 to N-1, whener N is a hyperparameter) such that all the contraint equations are satisified and a cost function J(a,delta) is minimized. Since our objective is to force the cte and epsi to 0 while maintaining a refernce velocity vref around the track, the cost function I used contains weighted quadratic terms penalizing these errors. It also contains some additional terms to tune performance:

* A term penalizes the use of delta for higher speeds
* A term penalizes velocity for high curvature in the waypoints (in other words, slows down for turns)
* A term minimizes the use of actuators
* A term minimizes the changes in actuator values between timesteps to encourage smooth transitions.

The above additions to the cost function were chosen after considerable trial and error.

![Alt text](images/cost.png?raw=true "Cost Function To Minimize")

* Timestep and Elapsed Duration: Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The values N and dt are hyperparameters to be chosen by the control system designer (i.e., me) to optimize performance. The objective to control the car along waypoints at most a second or two ahead in time. There are tradeoffs involved: too many waypoints (N) and teh computation time becomes problematic, while too few waypoints reduces the accuracy and optimality of the solution. Similarly, a dt too large will result is more state equation propagation error while a dt too small will not predict far enough ahead and will result in suboptimal solutions. I tried a number of values ranging from N=8 to N=30 and from dt=0.05 to 0.25. Since my MacBook can compute the optimal solution fairly quickly (typically in less than 0.05 seconds) I chose the following values to use: N=30, dt=0.05. These values result in using rpedicted waypoints from the currenttime unti approximately 1.5 seconds into the future.

* Polynomial Fitting and MPC Preprocessing: A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The waypoints are provided by the simulation in world coordinates. These represent the desired trajectory to be traveled by the car over the next 1-2 seconds. I transformed these waypoints in the car frame of reference, and then used the provied polyfit function to fit these x,y coordinates (in car coordinates) into a 3rd order polynomial f(x)=c0 + c1 * x + c2 * x * x + c3 * x * x * x. This polynomial function was used to compute the cte and epsi errors as part of the state equations.

* Model Predictive Control with Latency: The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Since any control system, especially a more complicated control system that runs an iterative optimization algorithm, requires a finite non-zero amount of time for computation, such computation delays can add lag (destabilizing) to the controller dynamics and therefore must be accounted for. The solution I used is as follows. At each controller timestep, we assume that the required computation/lag time will be 100ms. In other words, whatever control actions we calculate will not be applied until 100ms into the future. Therefore, at each control step I took the current vehicle state in car coordinates and used the equations of motion along with current control values to propage the state forward in time by 100ms. I then used this propagated state as the "initial" state for optimization calculations. That way, the controls I compute are the optima controls at the propagated state which is the state the car will be in by the time I can apply the controls in the first place. This is a proper solution to the latency issue if the actual latency is equal to the assumed latency used for propagating the initial equations forward in time.

* The vehicle must successfully drive a lap around the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). The car can't go over the curb, but, driving on the lines before the curb is ok.

The code in this repo successfully drives a car around the simulated track with a reference velocity of 80 MPH. The MPC system successfully navigates all curves and avoids curbs and any other off-road or unsafe areas. The included video demonstrates a couple of laps around the track.

# Original Readme

The remainer of this readme file consists of the original readme provided as part of the project.

---------------------------------------------------

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
