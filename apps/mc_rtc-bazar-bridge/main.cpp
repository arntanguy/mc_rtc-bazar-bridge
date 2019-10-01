#include <fri/kuka_lwr_driver.h>
#include <fri/kuka_lwr_robot.h>
#include <ati/force_sensor_driver.h>

#include <pid/signal_manager.h>

#include <iostream>

int main(int argc, const char** argv) {
    const double cycle_time_s{0.005}; // Can be set between 1 and 20ms
    const int left_arm_port{49938};   // local UDP port (see KRC configuration)
    const int right_arm_port{49939};  // local UDP port (see KRC configuration)

    std::string left_ft_name{"FT07321"};
    std::string right_ft_name{"FT21001"};
    double cutoff_freq_hz{20}; // Can be set to -1 to use the highest possible
                               // one without aliasing

    fri::KukaLWRRobot left_arm{};
    fri::KukaLWRRobot right_arm{};

    fri::KukaLWRDriver left_arm_driver{left_arm,
                                       fri::KukaLWRDriver::JointPositionControl,
                                       cycle_time_s, left_arm_port};

    fri::KukaLWRDriver right_arm_driver{
        right_arm, fri::KukaLWRDriver::JointPositionControl, cycle_time_s,
        right_arm_port};

    ati::ForceSensorDriver dual_force_sensor_driver{
        {{"ati_calibration_files/" + left_ft_name + ".cal",
          ati::ForceSensorDriver::Port::First},
         {"ati_calibration_files/" + right_ft_name + ".cal",
          ati::ForceSensorDriver::Port::Second}},
        ati::ForceSensorDriver::OperationMode::AsynchronousAcquisition,
        1. / cycle_time_s, // Update frequency (sampling frequency is always set
                           // to 25kHz)
        cutoff_freq_hz};   // Filter cutoff frequency, <0 to disable it (applies
                           // to the 25kHz data)

    dual_force_sensor_driver.readOffsets(
        "ati_offset_files/" + left_ft_name + ".yaml", 0);
    dual_force_sensor_driver.readOffsets(
        "ati_offset_files/" + right_ft_name + ".yaml", 1);

    const auto& left_wrench =
        dual_force_sensor_driver.getWrench(0); // First port

    const auto& right_wrench =
        dual_force_sensor_driver.getWrench(1); // Second port

    if (not dual_force_sensor_driver.init()) {
        std::cerr << "Failed to initialize the F/T driver" << std::endl;
        std::exit(-1);
    }

    if (not left_arm_driver.init()) {
        std::cerr << "Failed to initialize the left arm driver" << std::endl;
        std::exit(-1);
    }

    if (not right_arm_driver.init()) {
        std::cerr << "Failed to initialize the right arm driver" << std::endl;
        std::exit(-1);
    }

    bool stop{false};
    pid::SignalManager::registerCallback(
        pid::SignalManager::Interrupt, // Catch ctrl-c to quit properly
        "stop", [&stop](int) { stop = true; });

    left_arm.command.joint_position = left_arm.state.joint_position;
    right_arm.command.joint_position = right_arm.state.joint_position;

    while (not stop) {
        left_arm_driver.wait_For_KRC_Tick(); // Wait for synchronization signal

        left_arm_driver.get_Data();
        right_arm_driver.get_Data();
        dual_force_sensor_driver.process();

        // TODO: send sensor data to mc_rtc

        std::cout << "---------------------------------------\n";
        std::cout << "Left arm:\n";
        std::cout << "\tposition: ";
        std::cout << left_arm.state.joint_position.transpose() << '\n';
        std::cout << "\twrench: ";
        std::cout << left_wrench << '\n';

        std::cout << "Right arm:\n";
        std::cout << "\tposition: ";
        std::cout << right_arm.state.joint_position.transpose() << '\n';
        std::cout << "\twrench: ";
        std::cout << right_wrench << '\n';

        // TODO: read command data to mc_rtc
        // left_arm.command.joint_position = ...;
        // right_arm.command.joint_position = ...;

        left_arm_driver.send_Data();
        right_arm_driver.send_Data();
    }

    if (not dual_force_sensor_driver.end()) {
        std::cerr << "Failed to stop the F/T arm driver" << std::endl;
    }

    if (not left_arm_driver.end()) {
        std::cerr << "Failed to stop the left arm driver" << std::endl;
    }

    if (not right_arm_driver.end()) {
        std::cerr << "Failed to stop the right arm driver" << std::endl;
    }
}