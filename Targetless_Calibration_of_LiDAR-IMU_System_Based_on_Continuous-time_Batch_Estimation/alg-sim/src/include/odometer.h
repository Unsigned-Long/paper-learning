//
// Created by csl on 9/20/22.
//

#ifndef ALG_SIM_ODOMETER_H
#define ALG_SIM_ODOMETER_H

#include "sensor.h"

namespace ns_calib {
    template<class SensorType>
    class Odometer {
    public:
        using Sensor = SensorType;
        using SensorPtr = std::shared_ptr<Sensor>;

    public:
        SensorPtr sensor;

    public:
        explicit Odometer(const SensorPtr &sensor) : sensor(sensor) {}
    };


}


#endif //ALG_SIM_ODOMETER_H
