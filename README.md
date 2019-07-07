# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.
```code
// The max s value before wrapping around the track back to 0
double max_s = 6945.554;
```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric Goals description

### 1. The car is able to drive at least 4.32 miles without incident
The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

![succeeded trip](https://github.com/MyadaRoshdi/Path-Planning/blob/master/img/done.png)

### 2. The car drives according to the speed limit.
To achieve car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffi, a max speed value is used which is slightly less than the speed limit( 50mph)

```code
double max_speed = 49.5;
```

### 3. Max Acceleration and Jerk are not Exceeded.
The car should not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3, first using an acceleration step : 
```code
double acc = 0.224;
```
and use a `ref_vel` value, which starts in the begining of the trip with 0, then updated with increasing or decreasing acceleration steps `acc` based on the car in front. the following conditions handles also if the car is in the middle lane `2` and lane changing.
```code
if (too_close) {
// A car is ahead
// Decide to shift lanes or slow down
if (!car_right && lane < 2) {
// No car to the right AND there is a right lane -> shift right
					lane++;
				} else if (!car_left && lane > 0) {
// No car to the left AND there is a left lane -> shift left
					lane--;
				} else {
// Nowhere to shift -> slow down
					ref_vel -= acc;
				}
			} else {
if (lane != 1) {
// Not in the center lane. Check if it is safe to move back
if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) {
// Move back to the center lane
						lane = 1;
					}
				}
if (ref_vel < max_speed) {
// No car ahead AND we are below the speed limit -> speed limit
					ref_vel += acc;
				}
			}
```
### 4. Car does not have collisions.
To acheive that, a gap distance is defined (value is impirically chosen):
```code
int gap = 50; // meters
// Identify whether the car is ahead, to the left, or to the right
if (car_lane == lane) {
// Another car is ahead
					too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < gap);
				} else if (car_lane - lane == 1) {
// Another car is to the right
					car_right |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
				} else if (lane - car_lane == 1) {
// Another car is to the left
					car_left |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
				}
			}
```
So after the above block of code, not if the gap is less than 50-m, they consider the car is too close, so an action is to deccelrate and change lanes which is handled in the above point (3.).
### 5. The car stays in its lane, except for the time between changing lanes.
The preferences is always, keep the car in the middle lane(i.e. lane 1), so the following condition is used to check if the lane is not currently in middle lane and it's safe to go back to the lane:

```code
if (lane != 1) {
// Not in the center lane. Check if it is safe to move back
if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) {
// Move back to the center lane
						lane = 1;
					}
				}
```
### 6. The car is able to change lanes
If distance with following are is too close, lane changing logic is implemented as follows:
```code
if (too_close) {
// A car is ahead
// Decide to shift lanes or slow down
if (!car_right && lane < 2) {
// No car to the right AND there is a right lane -> shift right
					lane++;
				} else if (!car_left && lane > 0) {
// No car to the left AND there is a left lane -> shift left
					lane--;
				} else {
// Nowhere to shift -> slow down
					ref_vel -= acc;
				}
			} else {
if (lane != 1) {
// Not in the center lane. Check if it is safe to move back
if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) {
// Move back to the center lane
						lane = 1;
					}
				}
```
## Reflections:
The implementation here is very specific to the given road structure. To generalize it on different lane structures, lots of work is needed (list of speed limits, list of road structure, road exit, traffic signs, .. etc). Also, method used here is logic programming which can do better if more smart decision is taked involving machine learning for path planning.
