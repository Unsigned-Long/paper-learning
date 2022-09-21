//
// Created by csl on 9/21/22.
//

#ifndef ALG_SIM_VISUAL_ODOMETER_H
#define ALG_SIM_VISUAL_ODOMETER_H

#include "odometer/odometer.h"
#include "sensor/sensor.h"

namespace ns_calib {
    class VisualOdometer : public Odometer<MonoPinholeCamera> {
    public:
        using Ptr = std::shared_ptr<VisualOdometer>;
    private:

    public:
        explicit VisualOdometer(const Sensor &sensor);

        static Ptr create(const Sensor &sensor);

        bool feedObservation(Sensor::ObvPtr obv);
    };
}


#endif //ALG_SIM_VISUAL_ODOMETER_H
