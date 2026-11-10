// ======================================================================
// \title  CameraComponent.cpp
// \author townsa
// \brief  cpp file for CameraComponent component implementation class
// ======================================================================

#include "CubesatRef/Components/CameraComponent/CameraComponent.hpp"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <cerrno>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace CubesatRef {

// ----------------------------------------------------------------------
// Helper: execute a command and wait for completion
// ----------------------------------------------------------------------

static bool runCommand(char* const args[]) {

    pid_t pid = fork();

    if (pid == 0) {
        execvp(args[0], args);
        _exit(127);
    }

    if (pid < 0) {
        return false;
    }

    int status = 0;
    pid_t result;

    do {
        result = waitpid(pid, &status, 0);
    } while (result < 0 && errno == EINTR);

    return (
        result == pid &&
        WIFEXITED(status) &&
        WEXITSTATUS(status) == 0
    );
}

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

CameraComponent::CameraComponent(const char* const compName)
    : CameraComponentComponentBase(compName) {}

CameraComponent::~CameraComponent() {}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void CameraComponent::TAKE_PHOTO_cmdHandler(
    FwOpcodeType opCode,
    U32 cmdSeq,
    U32 cameraId,
    const Fw::CmdStringArg& filePath
) {

    // STEP 1: Select camera channel

    const char* channelValue = nullptr;
    std::string cameraName;

    switch (cameraId) {

        case 0:
            channelValue = "0x02";
            cameraName = "camera0";
            break;

        case 1:
            channelValue = "0x12";
            cameraName = "camera1";
            break;

        case 2:
            channelValue = "0x22";
            cameraName = "camera2";
            break;

        case 3:
            channelValue = "0x32";
            cameraName = "camera3";
            break;

        case 5:
            channelValue = "0x00";
            cameraName = "camera_all";
            break;

        default:
            this->cmdResponse_out(
                opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR
            );
            return;
    }

    // STEP 2: Configure CamArray using I2C

    char* i2cArgs[] = {
        const_cast<char*>("sudo"),
        const_cast<char*>("-n"),
        const_cast<char*>("i2cset"),
        const_cast<char*>("-y"),
        const_cast<char*>("10"),
        const_cast<char*>("0x24"),
        const_cast<char*>("0x24"),
        const_cast<char*>(channelValue),
        nullptr
    };

    if (!runCommand(i2cArgs)) {
        this->cmdResponse_out(
            opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR
        );
        return;
    }

    // STEP 3: Get current Raspberry Pi time

    std::time_t currentTime = std::time(nullptr);
    std::tm* timeInfo = std::localtime(&currentTime);

    if (timeInfo == nullptr) {
        this->cmdResponse_out(
            opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR
        );
        return;
    }

    // STEP 4: Format timestamp

    char timestamp[32];

    std::strftime(
        timestamp,
        sizeof(timestamp),
        "%Y%m%d_%H%M%S",
        timeInfo
    );

    // STEP 5: Generate filename

    std::string filename =
        cameraName + "_" + timestamp + ".jpg";

    std::string directory = filePath.toChar();

    if (!directory.empty() && directory.back() != '/') {
        directory += "/";
    }

    std::string imagePath = directory + filename;

    // STEP 6: Log capture request

    Fw::LogStringArg eventPath(imagePath.c_str());

    this->log_ACTIVITY_HI_TakePhotoEvent(cameraId, eventPath);

    // STEP 7: Capture image

    char* cameraArgs[] = {
        const_cast<char*>("rpicam-still"),
        const_cast<char*>("-o"),
        const_cast<char*>(imagePath.c_str()),
        const_cast<char*>("--width"),
        const_cast<char*>("640"),
        const_cast<char*>("--height"),
        const_cast<char*>("480"),
        const_cast<char*>("--timeout"),
        const_cast<char*>("3000"),
        nullptr
    };

    if (runCommand(cameraArgs)) {

        // Capture successful
        this->tlmWrite_PhotoCount(++this->m_photoCount);

        this->cmdResponse_out(
            opCode, cmdSeq, Fw::CmdResponse::OK
        );

    } else {

        this->cmdResponse_out(
            opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR
        );
    }
}

} // namespace CubesatRef