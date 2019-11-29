#include <fri/kuka_lwr_driver.h>
#include <fri/kuka_lwr_robot.h>
#include <ati/force_sensor_driver.h>
#include <mpo700/MPO700interface.h>
#include <flir/driver.h>

#include <mc_udp/logging.h>
#include <mc_udp/server/Server.h>

#include <pid/signal_manager.h>

#include <CLI11/CLI11.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <unistd.h>

int main(int argc, const char** argv) {
    CLI::App app{"mc_rtc-bazar-bridge"};

    int port{4444};
    double cycle_time_s{0.005};        // Can be set between 1 and 20ms
    int left_arm_port{49938};          // local UDP port (see KRC configuration)
    int right_arm_port{49939};         // local UDP port (see KRC configuration)
    int mobile_base_local_port{22211}; // local UDP port
    int mobile_base_robot_port{22221}; // remote UDP port
    bool enable_mobile_base{false};
    std::string mobile_base_network_interface{"enp4s0f1"};
    std::string mobile_base_ip{"192.168.0.1"};

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
    add_option("enable_mobile_base", enable_mobile_base,
               "Enable control of the mobile base");

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

    mc_udp::Server server_sensor{port};
    mc_udp::Server server_control{port + 1};
    auto& sensors = server_sensor.sensors();
    const auto& control = server_control.control();
    sensors.encoders.resize(bazar_joint_count);
    sensors.encoderVelocities.resize(0); // no velocity feedback at the moment
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
    zero_array(sensors.floatingBasePos);
    zero_array(sensors.floatingBaseRPY);
    zero_array(sensors.floatingBaseVel);
    zero_array(sensors.floatingBaseAcc);

    server_sensor.sensors().id = 0;

    mpo700::MPO700CartesianState mobile_base_state;
    mpo700::MPO700CartesianVelocity mobile_base_command;

    std::optional<mpo700::MPO700Interface> mobile_base_driver;
    std::thread mobile_base_reception_thread;
    bool stop_mobile_base_reception_thread{false};

    if (enable_mobile_base) {
        mobile_base_driver.emplace(mobile_base_network_interface,
                                   mobile_base_ip, mobile_base_local_port,
                                   mobile_base_robot_port);

        if (not mobile_base_driver->init()) {
            std::cerr << "Failed to initialize the mobile base driver"
                      << std::endl;
            std::exit(-1);
        }

        mobile_base_reception_thread = std::thread([&]() {
            while (mobile_base_driver->update_State() and
                   not stop_mobile_base_reception_thread) {
                continue;
            }
            if (not stop_mobile_base_reception_thread) {
                std::cerr << "Failed to update state from the mobile base"
                          << std::endl;
            }
        });

        mobile_base_driver->consult_State(true);
        mobile_base_driver->exit_Command_Mode();

        if (not mobile_base_driver->enter_Cartesian_Command_Mode()) {
            std::cerr << "Cannot switch the mobile base to cartesian control"
                      << std::endl;
            std::exit(-1);
        }
    }

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

    flir::PanTilt ptu;
    flir::Driver pan_tilt_driver(ptu, "tcp:192.168.0.101");

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

    if (not pan_tilt_driver.start(flir::CommandMode::Position)) {
        std::cerr << "Failed to initialize the pan/tilt driver" << std::endl;
        std::exit(-1);
    } else {
        pan_tilt_driver.sync();
        pan_tilt_driver.read();
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
        sensors.floatingBasePos[0] = mobile_base_state.velocity.x_vel;
        sensors.floatingBasePos[1] = mobile_base_state.velocity.y_vel;
        sensors.floatingBaseRPY[2] = mobile_base_state.velocity.rot_vel;
        offset += base_joint_count;

        copy_n(left_arm.state.joint_position.data(), arm_joint_count,
               begin(sensors.encoders) + offset);
        offset += arm_joint_count;

        copy_n(right_arm.state.joint_position.data(), arm_joint_count,
               begin(sensors.encoders) + offset);
        offset += arm_joint_count;

        sensors.encoders[offset] = ptu.state.position.pan;
        sensors.encoders[offset + 1] = ptu.state.position.tilt;

        set_wrench("lhsensor", left_wrench);
        set_wrench("rhsensor", right_wrench);

        server_sensor.recv();
        server_sensor.send();
    };

    auto read_commands = [&]() {
        server_control.send();
        if (server_control.recv()) {
            // if (control.id != sensors.id) {
            //     MC_UDP_WARNING("[BAZAR] Server control id "
            //                    << control.id << " does not match sensors id "
            //                    << sensors.id)
            // } else {
            using namespace std;

            size_t offset{0};

            // Threhold on the mobile base velocities to avoid unnecessary
            // repositionning of the wheels
            auto threshold = [](auto value) {
                if (std::abs(value) < 1e-6) {
                    return 0.;
                } else {
                    return value;
                }
            };

            const auto& velocities = control.encoderVelocities;
            // The udp client won't send joint velocities if not explicitely
            // asked to
            if (not velocities.empty()) {
                mobile_base_command.x_vel = threshold(velocities[0]);
                mobile_base_command.y_vel = threshold(velocities[1]);
                mobile_base_command.rot_vel = threshold(velocities[2]);
            } else {
                mobile_base_command.x_vel = 0.;
                mobile_base_command.y_vel = 0.;
                mobile_base_command.rot_vel = 0.;
            }
            offset += base_joint_count;

            copy_n(begin(control.encoders) + offset, arm_joint_count,
                   left_arm.command.joint_position.data());
            offset += arm_joint_count;

            copy_n(begin(control.encoders) + offset, arm_joint_count,
                   right_arm.command.joint_position.data());
            offset += arm_joint_count;

            ptu.command.position.pan = control.encoders[offset];
            ptu.command.position.tilt = control.encoders[offset + 1];

            sensors.id++;
        }
        // }
    };

    bool stop{false};
    pid::SignalManager::registerCallback(
        pid::SignalManager::Interrupt, // Catch ctrl-c to quit properly
        "stop", [&stop](int) { stop = true; });

    left_arm.command.joint_position = left_arm.state.joint_position;
    right_arm.command.joint_position = right_arm.state.joint_position;
    mobile_base_command.x_vel = 0;
    mobile_base_command.y_vel = 0;
    mobile_base_command.rot_vel = 0;
    ptu.command.position.pan = ptu.state.position.pan;
    ptu.command.position.tilt = ptu.state.position.tilt;
    ptu.command.velocity = ptu.limits.max_velocity;

    while (not stop) {
        bool all_ok = true;
        left_arm_driver.wait_For_KRC_Tick(); // Wait for synchronization signal

        all_ok &= left_arm_driver.get_Data();
        all_ok &= right_arm_driver.get_Data();
        all_ok &= dual_force_sensor_driver.process();
        if (mobile_base_driver) {
            mobile_base_driver->get_Cartesian_State(mobile_base_state);
        }
        all_ok &= pan_tilt_driver.read();

        if (not all_ok) {
            std::cerr << "Failed to update the state from the robot"
                      << std::endl;
            break;
        }

        send_state();

        read_commands();

        all_ok &= left_arm_driver.send_Data();
        all_ok &= right_arm_driver.send_Data();
        if (mobile_base_driver) {
            all_ok &=
                mobile_base_driver->set_Cartesian_Command(mobile_base_command);
        }
        all_ok &= pan_tilt_driver.send();

        if (not all_ok) {
            std::cerr << "Failed to send commands the robot" << std::endl;
            break;
        }
    }

    pid::SignalManager::unregisterCallback(pid::SignalManager::Interrupt,
                                           "stop");

    if (not dual_force_sensor_driver.end()) {
        std::cerr << "Failed to stop the F/T arm driver" << std::endl;
    }

    if (not left_arm_driver.end()) {
        std::cerr << "Failed to stop the left arm driver" << std::endl;
    }

    if (not right_arm_driver.end()) {
        std::cerr << "Failed to stop the right arm driver" << std::endl;
    }

    if (mobile_base_driver) {
        mobile_base_command.x_vel = 0.;
        mobile_base_command.y_vel = 0.;
        mobile_base_command.rot_vel = 0.;
        mobile_base_driver->set_Cartesian_Command(mobile_base_command);

        if (mobile_base_reception_thread.joinable()) {
            stop_mobile_base_reception_thread = true;
            mobile_base_reception_thread.join();
        }

        mobile_base_driver->exit_Command_Mode();
        mobile_base_driver->consult_State(false);
        mobile_base_driver->end();
    }

    if (not pan_tilt_driver.stop()) {
        std::cerr << "Failed to stop the pan/tilt driver" << std::endl;
    }
}
