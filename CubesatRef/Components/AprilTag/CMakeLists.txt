register_fprime_library(
    AUTOCODER_INPUTS "${CMAKE_CURRENT_LIST_DIR}/AprilTag.fpp"
    SOURCES "${CMAKE_CURRENT_LIST_DIR}/AprilTag.cpp"
    DEPENDS apriltag CubesatRef_Components_Types Fw_Logger
)
target_include_directories(${FPRIME_CURRENT_MODULE} SYSTEM PUBLIC
    "${CMAKE_SOURCE_DIR}/lib/apriltag" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${FPRIME_CURRENT_MODULE} PUBLIC ${OpenCV_LIBS})
