#include <fri/kuka_lwr_driver.h>
#include <fri/kuka_lwr_robot.h>
#include <ati/force_sensor_driver.h>

#include <mc_udp/logging.h>
#include <mc_udp/server/Server.h>

#include <pid/signal_manager.h>

#include <CLI11/CLI11.hpp>

#include <iostream>
#include <chrono>
#include <unistd.h>

int main(int argc, const char** argv) {
    CLI::App app{"mc_rtc-bazar-bridge"};

    int port{4444};
    double cycle_time_s{0.005}; // Can be set between 1 and 20ms
    int left_arm_port{49938};   // local UDP port (see KRC configuration)
    int right_arm_port{49939};  // local UDP port (see KRC configuration)

    std::string left_ft_serial{"FT07321"};
    std::string right_ft_serial{"FT21001"};
    double cutoff_freq_hz{20}; // Can be set to -1 to use the highest possible
                               // one without aliasing

    auto add_option = [&](const std::string& name, auto& value,
                          const std::string& desc) {
        std::stringstream ss;
        ss << desc << " [" << value << "]";
        app.add_option("--" + name, value, ss.str());
    };

    add_option("port", port, "The UDP port to bind to");
    add_option("cycle_time", cycle_time_s,
               "The Kuka LWR cycle time in seconds");
    add_option("left_arm_port", left_arm_port,
               "The Kuka LWR left arm UDP port");
    add_option("right_arm_port", right_arm_port,
               "The Kuka LWR right arm UDP port");
    add_option("left_ft_serial", left_ft_serial,
               "The left force/torque sensor serial number");
    add_option("right_ft_serial", right_ft_serial,
               "The right force/torque sensor serial number");
    add_option("ft_cutoff_freq", cutoff_freq_hz,
               "The force/torque sensors cutoff frequency in Hertz");

    CLI11_PARSE(app, argc, argv);

    if (app.count_all() == 1) {
        std::cout << "Using default parameters: run with --help to list the "
                     "parameters that can be changed"
                  << std::endl;
    }

    const size_t base_joint_count{3};
    const size_t arm_joint_count{7};
    const size_t pan_tilt_joint_count{2};
    const size_t bazar_joint_count{base_joint_count + 2 * arm_joint_count +
                                   pan_tilt_joint_count};

    mc_udp::Server server(port);
    auto& sensors = server.sensors();
    sensors.encoders.resize(bazar_joint_count);
    sensors.torques.resize(bazar_joint_count);

    auto zero_array = [](auto& array) {
        std::fill(std::begin(array), std::end(array), 0.);
    };

    {
        std::array<double, 6> ft_values{};
        zero_array(ft_values);
        sensors.fsensor("lhsensor", ft_values.data());
        sensors.fsensor("rhsensor", ft_values.data());
    }

    zero_array(sensors.orientation);
    zero_array(sensors.angularVelocity);
    zero_array(sensors.angularAcceleration);

    server.sensors().id = 0;

    fri::KukaLWRRobot left_arm{};
    fri::KukaLWRRobot right_arm{};

    fri::KukaLWRDriver left_arm_driver{left_arm,
                                       fri::KukaLWRDriver::JointPositionControl,
                                       cycle_time_s, left_arm_port};

    fri::KukaLWRDriver right_arm_driver{
        right_arm, fri::KukaLWRDriver::JointPositionControl, cycle_time_s,
        right_arm_port};

    ati::ForceSensorDriver dual_force_sensor_driver{
        {{"ati_calibration_files/" + left_ft_serial + ".cal",
          ati::ForceSensorDriver::Port::First},
         {"ati_calibration_files/" + right_ft_serial + ".cal",
          ati::ForceSensorDriver::Port::Second}},
        ati::ForceSensorDriver::OperationMode::AsynchronousAcquisition,
        1. / cycle_time_s, // Update frequency (sampling frequency is always set
                           // to 25kHz)
        cutoff_freq_hz};   // Filter cutoff frequency, <0 to disable it (applies
                           // to the 25kHz data)

    dual_force_sensor_driver.readOffsets(
        "ati_offset_files/" + left_ft_serial + ".yaml", 0);
    dual_force_sensor_driver.readOffsets(
        "ati_offset_files/" + right_ft_serial + ".yaml", 1);

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

    auto set_wrench = [&](const std::string& name, const ati::Wrench& wrench) {
        std::array<double, 6> ft_values{};
        std::copy_n(wrench.forces.data(), 3, begin(ft_values));
        std::copy_n(wrench.torques.data(), 3, begin(ft_values) + 3);
        sensors.fsensor(name, ft_values.data());
    };

    auto send_state = [&]() {
        using namespace std;

        size_t offset{0};

        fill_n(begin(sensors.encoders) + offset, base_joint_count, 0.);
        offset += base_joint_count;

        copy_n(left_arm.state.joint_position.data(), arm_joint_count,
               begin(sensors.encoders) + offset);
        offset += arm_joint_count;

        copy_n(right_arm.state.joint_position.data(), arm_joint_count,
               begin(sensors.encoders) + offset);
        offset += arm_joint_count;

        fill_n(begin(sensors.encoders) + offset, pan_tilt_joint_count, 0.);

        set_wrench("lhsensor", left_wrench);
        set_wrench("rhsensor", right_wrench);

        server.send();
    };

    auto read_commands = [&]() {
        if (server.recv()) {
            const auto& control = server.control();
            if (control.id != sensors.id) {
                MC_UDP_WARNING("[BAZAR] Server control id "
                               << server.control().id
                               << " does not match sensors id "
                               << server.sensors().id)
            } else {
                using namespace std;

                size_t offset{0};

                // TODO send base commands
                offset += base_joint_count;

                copy_n(begin(control.encoders) + offset, arm_joint_count,
                       left_arm.command.joint_position.data());
                offset += arm_joint_count;

                copy_n(begin(control.encoders) + offset, arm_joint_count,
                       right_arm.command.joint_position.data());
                offset += arm_joint_count;

                // TODO send pan tilt commands
            }
        }
        server.sensors().id += 1;
    };

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

        send_state();

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

        read_commands();

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