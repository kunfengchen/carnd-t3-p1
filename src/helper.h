//
// Created by kchen on 8/4/17.
//

#ifndef CARND_T3_P1_HELPER_H
#define CARND_T3_P1_HELPER_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

    /** JMT source code from Udacity: Implement Quintic Polynomial Solver Solution
     * @param start
     * @param end
     * @param T
     * @return
     */
    std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T) {
        /* Calculate the Jerk Minimizing Trajectory that connects the initial state
         * to the final state in time T.
         * INPUTS
         * start - the vehicles start location given as a length three array
         * corresponding to initial values of [s, s_dot, s_double_dot]
         * end   - the desired end state for vehicle. Like "start" this is a
         * length three array.
         * T     - The duration, in seconds, over which this maneuver should occur.
         * OUTPUT
         * an array of length 6, each value corresponding to a coefficent in the polynomial
         * s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
         *
         * EXAMPLE
         *
         * > JMT( [0, 10, 0], [10, 10, 0], 1)
         * [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
        */

        MatrixXd A = MatrixXd(3, 3);
        A << T*T*T, T*T*T*T, T*T*T*T*T,
                3*T*T, 4*T*T*T,5*T*T*T*T,
                6*T, 12*T*T, 20*T*T*T;

        MatrixXd B = MatrixXd(3,1);
        B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
                end[1]-(start[1]+start[2]*T),
                end[2]-start[2];

        MatrixXd Ai = A.inverse();

        MatrixXd C = Ai*B;

        std::vector <double> result = {start[0], start[1], .5*start[2]};
        for(int i = 0; i < C.size(); i++)
        {
            result.push_back(C.data()[i]);
        }

        return result;

    }

}
#endif //CARND_T3_P1_HELPER_H
