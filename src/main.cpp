#include <iostream>
#include <print>
#include "filter.hpp"
// include depthai library
#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <rerun.hpp>

struct EulerAngles {
        double roll, pitch, yaw;
};

template <> struct std::formatter<EulerAngles> : std::formatter<std::string> {
        constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

        template <typename FormatContext> auto format(const EulerAngles& e, FormatContext& ctx) const {
            return std::formatter<std::string>::format(std::format("Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}",
                                                                   e.roll * 180.0 / M_PI, e.pitch * 180.0 / M_PI,
                                                                   e.yaw * 180.0 / M_PI),
                                                       ctx);
        }
};

EulerAngles quat_to_euler(double x, double y, double z, double w) {
    EulerAngles euler {};

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    euler.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (w * y - x * z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    euler.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    euler.yaw = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

int main() {
    std::println("start");
    const auto rec = rerun::RecordingStream("imu_stream");
    rec.spawn().exit_on_failure();
    dai::Pipeline pipeline;
    auto imu = pipeline.create<dai::node::IMU>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    std::println("pipeline start");
    xlinkOut->setStreamName("imu");

    imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 400);
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER, 500);

    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline
    // with a lot of input/output connections above this threshold packets will be sent in batch of X, if the host is
    // not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to
    // multiple nodes
    imu->setMaxBatchReports(10);

    imu->out.link(xlinkOut->input);

    dai::Device d(pipeline);

    auto imuQueue = d.getOutputQueue("imu", 50, false);
    auto baseTs = std::chrono::steady_clock::now();

    while (true) {
        auto imuData = imuQueue->get<dai::IMUData>();

        auto imuPackets = imuData->packets;
        for (auto& imuPacket : imuPackets) {
            auto& rot_vector = imuPacket.rotationVector;
            auto& accel_vector = imuPacket.acceleroMeter;
            auto quat = rerun::Quaternion::from_xyzw(-rot_vector.i, -rot_vector.j, -rot_vector.k, -rot_vector.real);

            auto euler = quat_to_euler(-rot_vector.i, -rot_vector.j, -rot_vector.k, -rot_vector.real);

            auto rot = rerun::Rotation3D(quat);

            auto box = rerun::Boxes3D::from_centers_and_sizes({{0, 0, 0}}, {{1, 3, 1}})
                           .with_quaternions({quat})
                           .with_colors({rerun::Color(200, 50, 0)})
                           .with_fill_mode(rerun::FillMode::Solid);

            rec.log("imu/orientation", box);

            rec.log("imu/accel_x", rerun::Scalar(accel_vector.x));
            rec.log("imu/accel_y", rerun::Scalar(accel_vector.y));
            rec.log("imu/accel_z", rerun::Scalar(accel_vector.z));

            rec.log("imu/roll", rerun::Scalar(euler.roll));
            rec.log("imu/pitch", rerun::Scalar(euler.pitch));
            rec.log("imu/yaw", rerun::Scalar(euler.yaw));

            std::println("{}", euler);
        }

        int key = cv::waitKey(1);
        if (key == 'q') { return 0; }
    }

    return 0;
}
