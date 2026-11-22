// ======================================================================
// \title  CameraComponent.cpp
// \author townsa
// \brief  cpp file for CameraComponent component implementation class
// ======================================================================

#include "CubesatRef/Components/CameraComponent/CameraComponent.hpp"
#include <Fw/Types/StringTemplate.hpp>

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <cerrno>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace Managers {

// ----------------------------------------------------------------------
// Helper: execute a command and wait for completion
// returns the exit status (0 = success), 127 if the program couldn't be
// run, 128+N if killed by signal N, or -1 if fork/waitpid failed
// ----------------------------------------------------------------------

static I32 runCommand(char* const args[]) {

    pid_t pid = fork();

    if (pid == 0) {
        execvp(args[0], args);
        _exit(127);
    }

    if (pid < 0) {
        return -1;
    }

    int status = 0;
    pid_t result;

    do {
        result = waitpid(pid, &status, 0);
    } while (result < 0 && errno == EINTR);

    if (result != pid) {
        return -1;
    }
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    if (WIFSIGNALED(status)) {
        return 128 + WTERMSIG(status);
    }
    return -1;
}

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

CameraComponent::CameraComponent(const char* const compName)
    : CameraComponentComponentBase(compName) {}

CameraComponent::~CameraComponent() {}

// State machine action handlers

void CameraComponent::Managers_CameraComponentStateMachine_action_takePhoto(
    SmId smId, Managers_CameraComponentStateMachine::Signal signal) {
        
    // No special initialization needed for this component
    const char* channelValue = nullptr;
    std::string cameraName;
    std::string filePath = "/home/cfl740/Desktop/images"; // default directory for images
    U32 cameraId = 5; // default to all cameras

    channelValue = "0x00"; // default to all cameras
            cameraName = "camera_all";


     // STEP 2: Configure CamArray using I2C
    // no sudo: the FSW user needs to be in the i2c group for /dev/i2c-10 access

    char* i2cArgs[] = {
        const_cast<char*>("i2cset"),
        const_cast<char*>("-y"),
        const_cast<char*>("10"),
        const_cast<char*>("0x24"),
        const_cast<char*>("0x24"),
        const_cast<char*>(channelValue),
        nullptr
    };

    const I32 i2cStatus = runCommand(i2cArgs);
    if (i2cStatus != 0) {
        this->log_WARNING_HI_CameraStepFailed(Fw::LogStringArg("i2cset camera select"), i2cStatus);
        this->camSm_sendSignal_fault();
        return;
    }

    // STEP 3: Get current Raspberry Pi time
    std::time_t currentTime = std::time(nullptr);
    std::tm* timeInfo = std::localtime(&currentTime);

    if (timeInfo == nullptr) {
        this->log_WARNING_HI_CameraStepFailed(Fw::LogStringArg("localtime"), -1);
        this->camSm_sendSignal_fault();
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
    std::string filename = cameraName + "_" + timestamp + ".jpg";
    std::string directory = filePath;
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

    const I32 captureStatus = runCommand(cameraArgs);
    if (captureStatus == 0) {

        // Capture successful
        this->tlmWrite_PhotoCount(++this->m_photoCount);

        // Send the saved image path through the output port
        if (this->isConnected_imageCaptured_OutputPort(0)) {
            Fw::StringTemplate<512> savedImagePath(imagePath.c_str());
            this->imageCaptured_out(0, savedImagePath);
        }

    } 
    
    else {
        this->log_WARNING_HI_CameraStepFailed(Fw::LogStringArg("rpicam-still capture"), captureStatus);
    }
}

// retry the CamArray i2c select each tick; go back to RUNNING once the bus responds
void CameraComponent::Managers_CameraComponentStateMachine_action_doFaultRecovery(
    SmId smId, Managers_CameraComponentStateMachine::Signal signal) {
    FW_ASSERT(smId == SmId::camSm);

    char* i2cArgs[] = {
        const_cast<char*>("i2cset"),
        const_cast<char*>("-y"),
        const_cast<char*>("10"),
        const_cast<char*>("0x24"),
        const_cast<char*>("0x24"),
        const_cast<char*>("0x00"),
        nullptr
    };

    if (runCommand(i2cArgs) == 0) {
        this->camSm_sendSignal_success();
    }
}

void CameraComponent::run_handler(FwIndexType, U32) {
    this->camSm_sendSignal_tick();
}

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
    // no sudo: the FSW user needs to be in the i2c group for /dev/i2c-10 access

    char* i2cArgs[] = {
        const_cast<char*>("i2cset"),
        const_cast<char*>("-y"),
        const_cast<char*>("10"),
        const_cast<char*>("0x24"),
        const_cast<char*>("0x24"),
        const_cast<char*>(channelValue),
        nullptr
    };

    const I32 i2cStatus = runCommand(i2cArgs);
    if (i2cStatus != 0) {
        this->log_WARNING_HI_CameraStepFailed(Fw::LogStringArg("i2cset camera select"), i2cStatus);
        this->cmdResponse_out(
            opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR
        );
        return;
    }

    // STEP 3: Get current Raspberry Pi time

    std::time_t currentTime = std::time(nullptr);
    std::tm* timeInfo = std::localtime(&currentTime);

    if (timeInfo == nullptr) {
        this->log_WARNING_HI_CameraStepFailed(Fw::LogStringArg("localtime"), -1);
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

    const I32 captureStatus = runCommand(cameraArgs);
    if (captureStatus == 0) {

        // Capture successful
        this->tlmWrite_PhotoCount(++this->m_photoCount);

        // Send the saved image path through the output port
        if (this->isConnected_imageCaptured_OutputPort(0)) {
            Fw::StringTemplate<512> savedImagePath(imagePath.c_str());
            this->imageCaptured_out(0, savedImagePath);
        }

        this->cmdResponse_out(
            opCode, cmdSeq, Fw::CmdResponse::OK
        );

    } else {

        this->log_WARNING_HI_CameraStepFailed(Fw::LogStringArg("rpicam-still capture"), captureStatus);
        this->cmdResponse_out(
            opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR
        );
    }
}

} // namespace Managers