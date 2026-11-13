#include "CubesatRef/Components/AprilTag/AprilTag.hpp"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include <Fw/Logger/Logger.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>

namespace Vision {

namespace {
// Settings for the 320x240 test images, UPDATE FOR FLIGHT CAMERA
constexpr char CSV_PATH[] = "poses.csv";
constexpr F64 FX = 288.0;
constexpr F64 FY = 288.0;
constexpr F64 CX = 160.0;
constexpr F64 CY = 120.0;
constexpr F64 TAG_SIZE_METERS = 0.096;
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

    std::ofstream csv(CSV_PATH, std::ios::app);
    if (!csv) {
        Fw::Logger::log("AprilTag: cannot open %s\n", CSV_PATH);
        return false;
    }

    // AprilTag reads the OpenCV grayscale buffer directly.
    image_u8_t image = {gray.cols, gray.rows, static_cast<int>(gray.step), gray.data};
    zarray_t* detections = apriltag_detector_detect(m_detector, &image);

    csv << std::setprecision(17);
    if (csv.tellp() == 0) {
        csv << "frame,tag_id,tx,ty,tz,r11,r12,r13,r21,r22,r23,r31,r32,r33\n";
    }

    for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t* detection;
        zarray_get(detections, i, &detection);

        apriltag_detection_info_t info = {detection, TAG_SIZE_METERS, FX, FY, CX, CY};
        apriltag_pose_t pose;
        estimate_tag_pose(&info, &pose);

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
