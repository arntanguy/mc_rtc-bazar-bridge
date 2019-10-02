PID_Component(
    APPLICATION
    NAME mc_rtc-bazar-bridge
    DIRECTORY mc_rtc-bazar-bridge
    DEPEND 
        api-driver-fri/kuka-lwr-driver
        ati-force-sensor-driver/ati-force-sensor-driver
        pid-os-utilities/pid-signal-manager
        cli11/cli11
    INTERNAL LINKS
        mc_udp::mc_udp_server
    CXX_STANDARD 11
)