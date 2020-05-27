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

## Result

![alt text](../data/screenshot.png "Result")

## Code Overview

1. Given a lane number, a distance "horizon" to look forward and a distance "buffer" to look backward, Check whether in a vehicle is too close to ego vehicle. 
```
bool check_too_close(const vector<vector<double>>& sensor_fusion, const int& lane, const double& car_s, const int& prev_size, int horizon, int buffer) {

    for (size_t i = 0; i < sensor_fusion.size(); i++) {

        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
            // project the car s coordinates out to end of prev list
            check_car_s += (double)prev_size * 0.02 * check_speed;

            if ((check_car_s > car_s - buffer) && (check_car_s - horizon < car_s)) {
                return true;
            }
        }
    }

    return false;
}
```

2. Check is ego vehicle is too close to a front vehicle, if yes, whether a lane change is possible
```
bool too_close = false;
// Check if a car is in front of us
too_close = check_too_close(sensor_fusion, lane, car_s, prev_size, 30, 0);
// Check if lane change is possible
if (too_close) {
    if (lane == 0) {
        if (!check_too_close(sensor_fusion, lane + 1, car_s, prev_size, 35, 5)) {
            lane++;
        }
        // Check if a pass at right most lane is possible 
        else if (!check_too_close(sensor_fusion, lane + 2, car_s, prev_size, 35, 5)) {
            if (!check_too_close(sensor_fusion, lane + 1, car_s, prev_size, 20, 5)) {
                lane++;
            }
        }
    }
    else if (lane == 1) {
        if (!check_too_close(sensor_fusion, lane - 1, car_s, prev_size, 25, 5)) {
            lane--;
        }
        else if (!check_too_close(sensor_fusion, lane + 1, car_s, prev_size, 35, 5)) {
            lane++;
        }
    }
    else {
        if (!check_too_close(sensor_fusion, lane - 1, car_s, prev_size, 25, 5)) {
            lane--;
        }// Check if a pass at left most lane is possible 
        else if (!check_too_close(sensor_fusion, lane - 2, car_s, prev_size, 35, 5)) {
            if (!check_too_close(sensor_fusion, lane - 1, car_s, prev_size, 20, 5)) {
                lane--;
            }
        }
    }
}
```

3. Using some reference points to creat a set of points for spline (trajectory generation)
```
if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
}
else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

}

vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

4. Transfer the reference points from world coordinate to ego coordinate and put them to spline
```
// convert origin reference coordinates to ego in current frame
for (size_t i = 0; i < ptsx.size(); i++) {
double shift_x = ptsx[i] - ref_x;
double shift_y = ptsy[i] - ref_y;

ptsx[i] = (shift_x * cos(0 - ref_yaw)) - shift_y * sin(0 - ref_yaw);
ptsy[i] = (shift_x * sin(0 - ref_yaw)) + shift_y * cos(0 - ref_yaw);
}
// set spline
tk::spline s;
s.set_points(ptsx, ptsy);
```

5. Fill the trajectory up to 50 points with the points generated by spline.
```
// some horizon value in front of ego car
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(target_x * target_x + target_y * target_y);

double x_add_on = 0;

for (size_t i = 0; i < 50 - prev_size; i++) {
    // If it is too close to a front a vehicle, we slow down
    if (too_close) {
        ref_vel -= 0.15;
    } // If not we speed up to limit
    else if (ref_vel < 49.6) {
        ref_vel += 0.2;
    }
    //linearalization
    double N = target_dist / (0.02 * ref_vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    //transfer back to global coordinates
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
}
```