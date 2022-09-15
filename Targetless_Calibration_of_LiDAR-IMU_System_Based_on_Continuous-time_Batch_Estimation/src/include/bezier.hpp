//
// Created by csl on 9/15/22.
//

#ifndef TARGETLESS_CALIBRATION_OF_LIDAR_IMU_SYSTEM_BASED_ON_CONTINUOUS_TIME_BATCH_ESTIMATION_BEZIER_HPP
#define TARGETLESS_CALIBRATION_OF_LIDAR_IMU_SYSTEM_BASED_ON_CONTINUOUS_TIME_BATCH_ESTIMATION_BEZIER_HPP

#include "artwork/geometry/point.hpp"
#include "artwork/logger/logger.h"

namespace ns_spline {
    struct Bezier {
    public:
        static ns_geo::PointSet2f solve(const ns_geo::PointSet2f &controlPoints, std::size_t num) {
            float t = 0.0, delta = 1.0 / (num - 1);
            ns_geo::PointSet2f result(num);

            for (int i = 0; i < num; ++i, t += delta) {
                result[i] = bezier(t, controlPoints, 0, controlPoints.size());
            }
            return result;
        }

    protected:
        static ns_geo::Point2f bezier(float t, const ns_geo::PointSet2f &controlPoints,
                                      std::size_t beg, std::size_t end) {
            if (end - beg == 1) {
                return controlPoints[beg];
            }
            auto p1 = bezier(t, controlPoints, beg, end - 1);
            p1.x *= 1 - t;
            p1.y *= 1 - t;
            auto p2 = bezier(t, controlPoints, beg + 1, end);
            p2.x *= t;
            p2.y *= t;
            return ns_geo::Point2f(p1.x + p2.x, p1.y + p2.y);
        }
    };
}

#endif //TARGETLESS_CALIBRATION_OF_LIDAR_IMU_SYSTEM_BASED_ON_CONTINUOUS_TIME_BATCH_ESTIMATION_BEZIER_HPP
