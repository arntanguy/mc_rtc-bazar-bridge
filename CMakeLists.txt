CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the PID workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

project(mc_rtc-bazar-bridge)

PID_Package(
	AUTHOR      Benjamin Navarro
	YEAR        2019
	LICENSE     BSD
	DESCRIPTION Bridge to make mc_rtc talk to the BAZAR robot
	ADDRESS 	git@gite.lirmm.fr:cobot/mc_rtc-bazar-bridge.git
	README 		instructions.md
	CODE_STYLE	pid11
	VERSION     0.1.0
)

PID_Dependency(api-driver-fri)
PID_Dependency(ati-force-sensor-driver)
PID_Dependency(neobotix-mpo700-udp-interface)
PID_Dependency(flir-sdk)
PID_Dependency(pid-os-utilities)
PID_Dependency(cli11)

set(CMAKE_CXX_STANDARD 11)
find_package(mc_udp REQUIRED)

build_PID_Package()
