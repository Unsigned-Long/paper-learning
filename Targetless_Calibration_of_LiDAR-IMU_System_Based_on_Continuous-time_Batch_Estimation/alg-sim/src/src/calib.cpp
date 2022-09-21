//
// Created by csl on 9/20/22.
//
#include "calib.h"

namespace ns_calib {

    Status CalibSolver::solve() {
        Status status;

        status = initialization();
        checkSolveStatus(status);

        status = dataAssociation();
        checkSolveStatus(status);

        status = batchOptimization();
        checkSolveStatus(status);

        status = refinement();
        checkSolveStatus(status);

        status = Status(Status::Flag::FINE, "Solve Successfully.");
        return status;
    }

    Status CalibSolver::initialization() {
        Status status;

        return status;
    }

    Status CalibSolver::dataAssociation() {
        Status status;

        return status;
    }

    Status CalibSolver::batchOptimization() {
        Status status;

        return status;
    }

    Status CalibSolver::refinement() {
        Status status;

        return status;
    }

    void CalibSolver::checkSolveStatus(const Status &status) {
        switch (status.flag) {
            case Status::Flag::FINE:
            case Status::Flag::WARNING:
                std::cout << status << std::endl;
                break;
            case Status::Flag::ERROR:
            case Status::Flag::FETAL:
                throw status;
                break;
        }
    }

    CalibSolver::Ptr CalibSolver::create() {
        return std::make_shared<CalibSolver>();
    }

}

