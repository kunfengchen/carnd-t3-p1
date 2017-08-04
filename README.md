# carnd-t3-p1

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

## Third Party Library
* [spline tool](http://kluge.in-chemnitz.de/opensource/spline/)

## References
* (https://medium.com/@mohankarthik/path-planning-in-highways-for-an-autonomous-vehicle-242b91e6387d)
