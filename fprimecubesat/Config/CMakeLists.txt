####
# RadioTestDeployment configuration overrides
#   ComCfg.fpp: TmFrameFixedSize reduced to fit one LoRa packet (<= 255 bytes)
####
register_fprime_config(
    "RadioTestDeploymentConfig"
    CONFIGURATION_OVERRIDES
        "${CMAKE_CURRENT_LIST_DIR}/ComCfg.fpp"
        "${CMAKE_CURRENT_LIST_DIR}/ComCcsdsConfig.fpp"
        "${CMAKE_CURRENT_LIST_DIR}/AcConstants.fpp"
    INTERFACE
)