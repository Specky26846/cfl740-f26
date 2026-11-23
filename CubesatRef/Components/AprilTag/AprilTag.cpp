#include "CubesatRef/Components/AprilTag/AprilTag.hpp"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include <Fw/Logger/Logger.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iomanip>

namespace Vision {

namespace {
constexpr char CSV_PATH[] = "poses.csv";

// Camera 0, single-camera mode, calibrated at 640x480
constexpr F64 FX = 983.0738549582002;
constexpr F64 FY = 958.3931773136603;
constexpr F64 CX = 130.5908795240388;
constexpr F64 CY = 100.27366846146512;
constexpr F64 TAG_SIZE_METERS = 0.065;
}

AprilTag::AprilTag(const char* name) : AprilTagComponentBase(name) {
    m_family = tag36h11_create();
    m_detector = apriltag_detector_create();
    apriltag_detector_add_family(m_detector, m_family);
    m_detector->nthreads = 1;
    m_detector->quad_decimate = 1.0;
}

AprilTag::~AprilTag() {
    apriltag_detector_destroy(m_detector);
    tag36h11_destroy(m_family);
}

void AprilTag::PROCESS_IMAGE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                       const Fw::CmdStringArg& imagePath) {
    const bool success = this->processImage(imagePath.toChar());
    cmdResponse_out(opCode, cmdSeq, success ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

void AprilTag::processImage_handler(FwIndexType portNum, const Fw::StringBase& imagePath) {
    (void)portNum;
    (void)this->processImage(imagePath.toChar());
}

bool AprilTag::processImage(const char* imagePath) {
    cv::Mat gray = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
        Fw::Logger::log("AprilTag: cannot read image %s\n", imagePath);
        return false;
    }

    // Lens distortion coefficients in k1, k2, p1, p2, k3 order
    constexpr F64 K1 = -0.4225254762255126;
    constexpr F64 K2 = 0.40528981951368803;
    constexpr F64 P1 = 0.01736466886094117;
    constexpr F64 P2 = -0.02578802961941483;
    constexpr F64 K3 = 1.7693540422569356;

    // Remove lens distortion before detecting tags. Equivalent to
    // cv::undistort(gray, out, K, D, K), but built from imgproc only because
    // the RPi sysroot's OpenCV has no calib3d module. The map is rebuilt
    // only when the image size changes.
    static cv::Mat mapX, mapY;
    if (mapX.size() != gray.size()) {
        mapX.create(gray.size(), CV_32FC1);
        mapY.create(gray.size(), CV_32FC1);
        for (int v = 0; v < gray.rows; ++v) {
            float* mx = mapX.ptr<float>(v);
            float* my = mapY.ptr<float>(v);
            const F64 y = (v - CY) / FY;
            for (int u = 0; u < gray.cols; ++u) {
                const F64 x = (u - CX) / FX;
                const F64 r2 = x * x + y * y;
                const F64 radial = 1.0 + r2 * (K1 + r2 * (K2 + r2 * K3));
                const F64 xd = x * radial + 2.0 * P1 * x * y + P2 * (r2 + 2.0 * x * x);
                const F64 yd = y * radial + P1 * (r2 + 2.0 * y * y) + 2.0 * P2 * x * y;
                mx[u] = static_cast<float>(FX * xd + CX);
                my[u] = static_cast<float>(FY * yd + CY);
            }
        }
    }
    cv::Mat corrected;
    cv::remap(gray, corrected, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    gray = corrected;

    std::ofstream csv(CSV_PATH, std::ios::app);
    if (!csv) {
        Fw::Logger::log("AprilTag: cannot open %s\n", CSV_PATH);
        return false;
    }

    // AprilTag reads the OpenCV grayscale buffer directly
    image_u8_t image = {gray.cols, gray.rows, static_cast<int>(gray.step), gray.data};
    zarray_t* detections = apriltag_detector_detect(m_detector, &image);

    csv << std::setprecision(17);
    if (csv.tellp() == 0) {
        csv << "frame,tag_id,tx,ty,tz,r11,r12,r13,r21,r22,r23,r31,r32,r33\n";
    }
    
    // Start with no valid pose
    Types::TagPose poseData;
    poseData.set_valid(false);

    for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t* detection;
        zarray_get(detections, i, &detection);

        apriltag_detection_info_t info = {detection, TAG_SIZE_METERS, FX, FY, CX, CY};
        apriltag_pose_t pose;
        estimate_tag_pose(&info, &pose);

        // Use the first detected tag for the beacon
        if (i == 0) {
            poseData.set_tagId(static_cast<U16>(detection->id));
            poseData.set_valid(true);

            poseData.set_tx(static_cast<F32>(MATD_EL(pose.t, 0, 0)));
            poseData.set_ty(static_cast<F32>(MATD_EL(pose.t, 1, 0)));
            poseData.set_tz(static_cast<F32>(MATD_EL(pose.t, 2, 0)));

            poseData.set_r11(static_cast<F32>(MATD_EL(pose.R, 0, 0)));
            poseData.set_r12(static_cast<F32>(MATD_EL(pose.R, 0, 1)));
            poseData.set_r13(static_cast<F32>(MATD_EL(pose.R, 0, 2)));
            poseData.set_r21(static_cast<F32>(MATD_EL(pose.R, 1, 0)));
            poseData.set_r22(static_cast<F32>(MATD_EL(pose.R, 1, 1)));
            poseData.set_r23(static_cast<F32>(MATD_EL(pose.R, 1, 2)));
            poseData.set_r31(static_cast<F32>(MATD_EL(pose.R, 2, 0)));
            poseData.set_r32(static_cast<F32>(MATD_EL(pose.R, 2, 1)));
            poseData.set_r33(static_cast<F32>(MATD_EL(pose.R, 2, 2)));
        }

        csv << m_frame << ',' << detection->id;
        for (int row = 0; row < 3; ++row) {
            csv << ',' << MATD_EL(pose.t, row, 0);
        }
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                csv << ',' << MATD_EL(pose.R, row, col);
            }
        }
        csv << '\n';

        // Release each pose before processing the next tag.
        matd_destroy(pose.R);
        matd_destroy(pose.t);
    }

    
    poseData.set_timestamp(this->getTime().getSeconds());

    // Send the result through the pose output port
    if (this->isConnected_poseOut_OutputPort(0)) {
        this->poseOut_out(0, poseData);
    }
    apriltag_detections_destroy(detections);
    csv.close();
    if (!csv) {
        Fw::Logger::log("AprilTag: cannot write %s\n", CSV_PATH);
        return false;
    }
    ++m_frame;
    return true;
}

}
