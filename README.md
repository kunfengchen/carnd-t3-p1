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

## Third Party Library
* [spline tool](http://kluge.in-chemnitz.de/opensource/spline/)
