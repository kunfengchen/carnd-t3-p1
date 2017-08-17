# carnd-t3-p1
CarND Path Planning Project

## Starter Code Cloned from Udacity
https://github.com/udacity/CarND-Path-Planning-Project

## Simulator
You can donwload the simulator here: https://github.com/udacity/self-driving-car-sim/releases



## Generating the path
### Fisrt of all, generate the path flowing the map waypoints
* Creates the spline from the map waypoints
```
  tk::spline map_spline;
  int n = 80;
  vector<double> subx(map_waypoints_x.begin(), map_waypoints_x.begin() + n);
  vector<double> suby(map_waypoints_y.begin(), map_waypoints_y.begin() + n);

  map_spline.set_points(subx, suby);
```
* Creates next car positions to flow the map spline
```
for (int i = 0; i < 50 - path_size; i++) {
   pos_x += (dist_inc) * cos(angle);
   pos_y = map_spline(pos_x);
   next_x_vals.push_back(pos_x);
   next_y_vals.push_back(pos_y);
```
   _The car now is driving on the center of the road (on the doulbe yellow line!)_ 
* Make the car drives on the first inner lane (2 m away from the yellow line)
```
vector<double> pos_frenet;
vector<double> pos_lane;
for (int i = 0; i < 50 - path_size; i++) {
    pos_x += (dist_inc) * cos(angle);
    pos_y = map_spline(pos_x);
    cout << "new pos: (" << pos_x << ", " << pos_y << ")" << std::endl;
    pos_frenet = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
    cout << "new fre: [" << pos_frenet[0] << ", " << pos_frenet[1] << "]" << std::endl;
    pos_lane = getXY(pos_frenet[0], pos_frenet[1]+2,
    map_waypoints_s, map_waypoints_x, map_waypoints_y);
    next_x_vals.push_back(pos_lane[0]);
    next_y_vals.push_back(pos_lane[1]);
```
* Or Using Frenet coordinations to advance the distance
```
pos_lane = {pos_x, pos_y};
pos_frenet = getFrenet(pos_lane[0], pos_lane[1], angle, map_waypoints_x, map_waypoints_y);
double pos_s = pos_frenet[0];
double pos_d = 2; // pos_frenet[1];
for (int i = 0; i < 50 - path_size; i++) {
    pos_s += dist_inc;
		pos_lane = getXY(pos_s, pos_d,
                     map_waypoints_s, map_waypoints_x, map_waypoints_y);
    next_x_vals.push_back(pos_lane[0]);
    next_y_vals.push_back(pos_lane[1]);
```
Now the car can drive on inner lane, middle lane, and outside lane with pos_d equals to 2, 4, 10 respectively. For testing purpose, use a large number of dis_inc to finish a loop quicker. For example, dist_inc = 5 will have the car running 559 MPH, which the car will finish a loop in about 20 seconds.

### Keep distance from the car ahead
```
/// keep distance
auto lane_sensors = t3p1help::sortSensor(sensor_fusion);
double front_s = t3p1help::getFrontS(pos_s, pos_d, lane_sensors);
// cout << "front s dist: " << front_s << endl;

double total_path_size = 50;
double buffer = 10;
double steps = total_path_size - path_size;
double dist_inc;
dist_inc = (front_s - buffer)/steps;
if (dist_inc > max_dist_inc) {
    dist_inc = max_dist_inc;
}
```

The front_s will get the distance of in Frent s from the car ahead in the same lane. dist_inc is adjusted accordingly to keep a buffer distance.
The car is not driving smooth due to the sparse waypoints provided by the [hightway_map.csv](data/highway_map.csv). Will use spilne.h to smooth the trajectory.

### Flowering Path Planning Walkthrough Video
#### Good ideas learn from the video
* Provided getFrent() and getXY() transform back and forth are not one-to-one mapped. There are transformation losese that cause webbing and jittering. Instea of transforming every points from Frent, better way to do it is choose few Frent herizon points and transform those to global coordinates using getXY(). And use the few global horizon pionts and transform them into car local coodinates. And then use the few local horizon points to setup the spline function for granunaer local way points, that follow the map.

* The resulting spline is already jerk free, provided that local way points is short enough.
* Next import idea is the distance for new local waypoints
  * The default number of herizon waypoints is 50. 
  * The delta time between waypoint is fixed 0.02 seconds.
  * The herizon distance is 30 meters

  * To caculate the step in x direction
  ```
  double steps_x = (horizon_dist/(p_state.point_dt*p_state.ref_velocity/t3p1help::MPS_TO_MPH));
  double step_x = horizon_x/steps_x;
  ```

  * To caculate the next local waypoints
  ```
  local_x += step_x;
  local_y = spline(local_x);
  ```
  Note that local_y is from the spline function that has the perfect curve.
  
* Increase speed by velocity_step = 0.224 when limit_velocity = 49.5 is not reached.
* Decrease speed by velocity_step when the car infront is too close.
* Very import tip
  * Need to compare the sesorsed car's location ahead of the time, where the new waypoints are generated.
  ```
  // The right time for generating the additional path;
  double time_ahead = pre_size * p_state.point_dt;
  ```
  * Get the sensored car's distance
  ```
  dist = car[5] - car_s;
  v_x = car[3]; // velocity on x
  v_y = car[4]; // velocity on y
  sen_speed = sqrt(v_x*v_x + v_y*v_y);

  dist += time_ahead * sen_speed;
  ```
  
### Behavior Planning
* Very basic implementation
  * Increase the speed by speed_step to speed limit
  * Keep the constant limit speed
  * If a car infront withn 30 meters (too close)
    * Decrease the speed, and
    * Change to next lane if safe to do so
      * If no safe lane to move to, stay in the same lane
  * The avearge Best Miles without incidents is 15 miles


## Third Party Library
* [spline tool](http://kluge.in-chemnitz.de/opensource/spline/)

## References
* [Path Planning Walkthrough Video](https://www.youtube.com/watch?v=3QP3hJHm4WM&feature=youtu.be)
