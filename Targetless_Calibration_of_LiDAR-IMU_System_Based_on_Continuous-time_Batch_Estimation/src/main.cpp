//
// Created by csl on 9/15/22.
//
#include "bezier.hpp"

void testBezier() {
    ns_geo::PointSet2f controlsPoints{
            {0, 0},
            {0, 1},
            {1, 2},
            {3, 0},
            {2, -3}
    };
    auto result = ns_spline::Bezier::solve(controlsPoints, 100);
    result.write("../output/bezier.txt", std::ios::out);
}

int main(int argc, char *argv[]) {
    ::testBezier();
    return 0;
}