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
        using SensorPtr = typename Sensor::Ptr;

        using Ptr = Odometer<Sensor>;

    public:
        Sensor sensor;

    public:
        explicit Odometer(const Sensor &sensor) : sensor(sensor) {}

        virtual bool feedObservation(typename Sensor::ObvPtr obv) = 0;
    };

    class LiDAROdometer : public Odometer<LiDAR> {
    public:
        explicit LiDAROdometer(const LiDAR &lidar);

        bool feedObservation(LiDAR::ObvPtr obv) override;
    };

    class VisualOdometer : public Odometer<MonoPinholeCamera> {
    public:
        explicit VisualOdometer(const MonoPinholeCamera &monoPinholeCamera);

        bool feedObservation(MonoPinholeCamera::ObvPtr obv) override;
    };

}


#endif //ALG_SIM_ODOMETER_H
