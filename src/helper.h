//
// Created by kchen on 8/4/17.
//

#ifndef CARND_T3_P1_HELPER_H
#define CARND_T3_P1_HELPER_H

#include <iostream>
#include <vector>

namespace t3p1help {

    std::vector<std::vector<std::vector<double>>>
    sortSensor(std::vector<std::vector<double>> sensors) {
        double d;
        std::vector<std::vector<double>> lane0, lane1, lane2;
        for (auto car : sensors) {
            d = car[6];
            if (d < 4) { // lane0 for inner lane
                lane0.push_back(car);
            } else if ( d < 8) { // lane1 for middle lane
                lane1.push_back(car);
            } else { // lane2 for outer lane
                lane2.push_back(car);
            }
        }
        return {lane0, lane1, lane2};
    }

    void print_sensors(std::vector<std::vector<double>> sensors) {
        for (auto car : sensors) {
            for (double info : car) {
                std::cout << info << ", ";
            }
            std::cout << std::endl;
        }
    }
}
#endif //CARND_T3_P1_HELPER_H
