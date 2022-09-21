//
// Created by csl on 9/20/22.
//

#ifndef ALG_SIM_CALIB_H
#define ALG_SIM_CALIB_H

#include <utility>
#include <ostream>
#include "utils/enum_cast.hpp"
#include "string"
#include "memory"
#include "iostream"

namespace ns_calib {
    struct Status : std::exception {
        enum class Flag {
            FINE, WARNING, ERROR, FETAL
        };
    public:
        Flag flag;
        std::string what;

        Status(Flag flag, std::string what)
                : flag(flag), what(std::move(what)) {}

        Status() : flag(Flag::FINE), what() {}

        friend std::ostream &operator<<(std::ostream &os, const Status &status) {
            os << "[" << EnumCast::enumToString(status.flag) << "]-[" << status.what << "]";
            return os;
        }
    };

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
