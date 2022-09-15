//
// Created by csl on 9/15/22.
//

#ifndef SPLINE_BSPLINE_HPP
#define SPLINE_BSPLINE_HPP

#include "artwork/geometry/point.hpp"
#include "artwork/logger/logger.h"

namespace ns_spline {
    struct UniformBSpline {
        static ns_geo::PointSet2f solve(ns_geo::PointSet2f controlPoints,
                                        std::size_t k, std::size_t num) {
            std::size_t n = controlPoints.size() - 1;
            std::size_t m = n + k + 1;
            float t = 0.0;

            std::vector<float> tAry(m + 1);
            for (int i = 0; i < m + 1; ++i) {
                if (i >= k && i <= m - 1 - k) {
                    tAry[i] = t;
                    t += 1.0;
                } else {
                    tAry[i] = t;
                }
            }
            LOG_VAR(tAry);

            ns_geo::PointSet2f result(num);
            float delta = tAry.back() / (num - 1.0);
            t = delta;
            delta = (tAry.back() - delta * 2.0f) / (num - 1.0);
            for (int j = 0; j < num; ++j, t += delta) {
                result[j].x = 0.0f, result[j].y = 0.0f;
                for (int i = 0; i <= n; ++i) {
                    float val = deBoor(i, k, t, tAry);
                    result[j].x += controlPoints[i].x * val;
                    result[j].y += controlPoints[i].y * val;
                }
            }
            return result;
        }

    protected:
        static float deBoor(std::size_t i, std::size_t k, float t, const std::vector<float> &tAry) {
            if (k == 0) {
                if (t >= tAry[i] && t < tAry[i + 1]) {
                    return 1.0f;
                } else {
                    return 0.0f;
                }
            }
            float v1 = 0.0f, v2 = 0.0f;
            if (tAry[i + k] != tAry[i]) {
                v1 = (t - tAry[i]) / (tAry[i + k] - tAry[i]) * deBoor(i, k - 1, t, tAry);
            }
            if (tAry[i + k + 1] != tAry[i + 1]) {
                v2 = (tAry[i + k + 1] - t) / (tAry[i + k + 1] - tAry[i + 1]) * deBoor(i + 1, k - 1, t, tAry);
            }
            return v1 + v2;
        }
    };
}

#endif //SPLINE_BSPLINE_HPP
