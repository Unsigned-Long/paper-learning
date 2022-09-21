//
// Created by csl on 9/20/22.
//

#ifndef ALG_SIM_IMU_BSPLINE_H
#define ALG_SIM_IMU_BSPLINE_H

#include "odometer/odometer.h"

namespace ns_calib {
    class IMUBSpline : public Odometer<IMU> {
    private:
        std::vector<Sensor::ObvPtr> raw_data;

    public:
        explicit IMUBSpline(const IMU &imu);

        bool feedObservation(IMU::ObvPtr obv);

        bool computeBSpline();

        [[nodiscard]] double startTimeStamp() const;

        [[nodiscard]] double endTimeStamp() const;
    };
}


#endif //ALG_SIM_IMU_BSPLINE_H
