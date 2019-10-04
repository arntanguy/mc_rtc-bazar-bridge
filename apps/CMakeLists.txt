PID_Component(
    APPLICATION
    NAME mc_rtc-bazar-bridge
    DIRECTORY mc_rtc-bazar-bridge
    DEPEND 
        api-driver-fri/kuka-lwr-driver
        ati-force-sensor-driver/ati-force-sensor-driver
        neobotix-mpo700-udp-interface/mpo700-interface
        flir-sdk/flir
        pid-os-utilities/pid-signal-manager
        cli11/cli11
    INTERNAL LINKS
        mc_udp::mc_udp_server
    CXX_STANDARD 17
)