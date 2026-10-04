#include <gtest/gtest.h>
#include "map_builder/imu_processor.h"

namespace {
struct Fixture {
    Config config;
    std::shared_ptr<IESKF> filter = std::make_shared<IESKF>();
    std::unique_ptr<IMUProcessor> processor;
    Fixture() {
        config.imu_init_num = 2;
        config.gravity_align = false;
        config.t_il.setZero();
        processor = std::make_unique<IMUProcessor>(config, filter);
        SyncPackage initial;
        initial.cloud_end_time = 1.0;
        for (double stamp : {0.99, 0.995})
            initial.imus.emplace_back(V3D(0, 0, 9.81), V3D::Zero(), stamp);
        EXPECT_TRUE(processor->initialize(initial));
        filter->x().t_wi = V3D(1, 0, 0);
        filter->x().v = V3D(1, 0, 0);
    }
    SyncPackage scan(double begin, double end, const V3D &omega) {
        SyncPackage package;
        package.cloud_start_time = begin;
        package.cloud_end_time = end;
        package.cloud.reset(new CloudType);
        for (int i = 0; i < 200; ++i) {
            double stamp = 1.0 + i * 0.005;
            if (stamp >= end) break;
            package.imus.emplace_back(V3D(0, 0, 9.81), omega, stamp);
        }
        for (double offset : {0.0, 0.001, 0.05, end - begin}) {
            const double stamp = begin + offset;
            const M3D rotation = Sophus::SO3d::exp(omega * (stamp - 1.0)).matrix();
            const V3D point = rotation.transpose() * (V3D(10, 2, 1) - V3D(stamp, 0, 0));
            PointType p;
            p.x = point.x(); p.y = point.y(); p.z = point.z();
            p.curvature = offset * 1000;
            package.cloud->push_back(p);
        }
        return package;
    }
};

void check(double begin, double end, const V3D &omega) {
    Fixture fixture;
    auto package = fixture.scan(begin, end, omega);
    fixture.processor->undistort(package);
    const V3D expected = Sophus::SO3d::exp(omega * (end - 1.0)).matrix().transpose() *
                         (V3D(10, 2, 1) - V3D(end, 0, 0));
    for (size_t i = 0; i < package.cloud->size(); ++i) {
        const auto &p = package.cloud->points[i];
        EXPECT_NEAR(p.x, expected.x(), 2e-5) << "point " << i;
        EXPECT_NEAR(p.y, expected.y(), 2e-5) << "point " << i;
        EXPECT_NEAR(p.z, expected.z(), 2e-5) << "point " << i;
    }
}
TEST(ImuDeskew, ContinuousTranslationIncludesScanStart) { check(1.0, 1.1, V3D::Zero()); }
TEST(ImuDeskew, SkippedScansPreserveTranslation) { check(1.3, 1.4, V3D::Zero()); }
TEST(ImuDeskew, SkippedScansPreserveRotationAndTranslation) { check(1.3, 1.4, V3D(0, 0, 0.8)); }
}  // namespace
