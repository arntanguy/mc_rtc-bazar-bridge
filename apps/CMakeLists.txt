PID_Component(
    APPLICATION
    NAME mc_rtc-bazar-bridge
    DIRECTORY mc_rtc-bazar-bridge
    DEPEND 
        api-driver-fri/kuka-lwr-driver
        ati-force-sensor-driver/ati-force-sensor-driver
        pid-os-utilities/pid-signal-manager
    CXX_STANDARD 11
)