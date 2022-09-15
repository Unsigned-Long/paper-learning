//
// Created by csl on 9/15/22.
//
#include "bezier.hpp"
#include "bspline.hpp"

const ns_geo::PointSet2f controlsPoints{
        {0, 0},
        {1, 2},
        {2, 0},
        {3, 2},
        {4, 0}
};

void testBezier() {
    auto result = ns_spline::Bezier::solve(controlsPoints, 100);
    result.write("../output/bezier.txt", std::ios::out);
}

void testUniformBSpline() {
    auto result = ns_spline::UniformBSpline::solve(controlsPoints, 3, 100);
    result.write("../output/uniformBSpline.txt", std::ios::out);
}


int main(int argc, char *argv[]) {
    controlsPoints.write("../output/controlPoints.txt", std::ios::out);
    ::testBezier();
    ::testUniformBSpline();
    return 0;
}