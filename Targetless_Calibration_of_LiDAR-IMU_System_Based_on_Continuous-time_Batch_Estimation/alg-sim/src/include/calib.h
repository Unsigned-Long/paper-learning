//
// Created by csl on 9/20/22.
//

#ifndef ALG_SIM_CALIB_H
#define ALG_SIM_CALIB_H

#include <ostream>
#include "utils/enum_cast.hpp"
#include "string"
#include "memory"
#include "iostream"
#include "utils/status.hpp"

namespace ns_calib {

    class CalibSolver {
    public:
        using Ptr = std::shared_ptr<CalibSolver>;

        CalibSolver() = default;

    private:
        // data, members
    public:
        static CalibSolver::Ptr create();

        Status solve();

    public:

        Status initialization();

        Status dataAssociation();

        Status batchOptimization();

        Status refinement();

    protected:

        static void checkSolveStatus(const Status &status);
    };
}

#endif //ALG_SIM_CALIB_H
