//
// Created by kchen on 8/4/17.
//

#ifndef CARND_T3_P1_HELPER_H
#define CARND_T3_P1_HELPER_H

#include <iostream>
#include <vector>

namespace t3p1help {
    const double MAX_S = 6914.14925765991;

    /**
     * Return the lane number
     * @param d The d of Frenet coordinates
     * @return lane number
     */
    int get_lane_from_d( double d) {
        int r;
        if (d < 4) { // inner lane
            r = 0;
        } else if ( d < 8) { // middle lane
            r = 1;
        } else { // outer lane
            r = 2;
        }
        return r;
    }

    /**
     * Sort the sensor fusion into lanes
     */
    std::vector<std::vector<std::vector<double>>>
    sortSensor(std::vector<std::vector<double>> sensors) {
        double d;
        std::vector<std::vector<double>> lane0, lane1, lane2;
        for (auto car: sensors) {
            d = car[6];
            switch (get_lane_from_d(d)) {
            case 0: // inner lane
                lane0.push_back(car);
                break;
            case 1: // middle lane
                lane1.push_back(car);
                break;
            case 2: // outer lane
                lane2.push_back(car);
                break;
            }
        }
        return {lane0, lane1, lane2};
    }

    /**
     * Print the sensor fusion data
     * @param sensors
     */
    void print_sensors(std::vector<std::vector<double>> sensors) {
        for (auto car: sensors) {
            for (double info : car) {
                std::cout << info << ", ";
            }
            std::cout << std::endl;
        }
    }

    /**
     * Get the Frenet s distance ahead
     * @param car_s
     * @param car_d
     * @param range
     * @return
     */
    double getFrontS(double car_s, double car_d,
                     const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        double min_dist = MAX_S;
        int car_lane = get_lane_from_d(car_d);
        double dist;
        for (auto car: lane_sensors[car_lane]) {
            dist = car[5] - car_s;
            if (dist < 0) { // the sensored car is behind
                if (dist < (50 - MAX_S)) {
                    // S looped around
                    dist += MAX_S;
                }
            }
            if (dist > 0) {
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
        }
        return min_dist;
    }
}
#endif //CARND_T3_P1_HELPER_H
