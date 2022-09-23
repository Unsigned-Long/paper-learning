//
// Created by csl on 9/21/22.
//

#ifndef ALG_SIM_LIDARODOMETER_H
#define ALG_SIM_LIDARODOMETER_H

#include "odometer/odometer.h"
#include "sensor/sensor.h"
#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/registration/ndt.h"

namespace ns_calib {

    class LiDAROdometer : public Odometer<LiDAR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Ptr = std::shared_ptr<LiDAROdometer>;

        using Obv = Sensor::Obv;
        using ObvPtr = Sensor::ObvPtr;

        using FrameData = Sensor::FrameData;
        using FrameDataPtr = Sensor::FrameDataPtr;

        using Point = FrameData::PointType;

        using Pose = OdometerPose<double>;

    private:
        // point cloud map with time [map time: the reference frame's time]
        ObvPtr map;
        std::vector<ObvPtr> rawObvVec;
        std::vector<std::size_t> keyObvIdx;
        std::vector<Pose> poseVec_frame_to_map;

        bool initialized;

        ObvPtr lastFrame;

        // filtering input scan to increase speed of registration
        pcl::ApproximateVoxelGrid<Point>::Ptr filter;
        // Normal Distributions Transform
        pcl::NormalDistributionsTransform<Point, Point>::Ptr ndt;

    public:
        explicit LiDAROdometer(const Sensor &sensor);

        static Ptr create(const Sensor &sensor);

        bool feedObservation(const ObvPtr &obv, const Pose &posePred_cur_to_last = Pose(), bool updateMapAlways = true);

        [[nodiscard]] const ObvPtr &getMap() const;

    protected:
        void initAVGFilter();

        void initNDT();

        FrameDataPtr filterFrameData(const FrameDataPtr &input);

        [[nodiscard]] bool checkMotion(const Pose &last_frame_to_map, const Pose &cur_frame_to_map) const;
    };
}


#endif //ALG_SIM_LIDARODOMETER_H
