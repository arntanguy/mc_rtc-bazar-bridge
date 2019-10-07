
Overview
=========

Bridge to make mc rtc talk to the BAZAR robot

How to start the robot
======================

Switch on the power strip inside the drawer then turn on the two Kuka controllers. Once the controllers are started (~10min), run the FRIGenericControl script on both of them. More information on how to operate the Kuka controllers [here](https://gite.lirmm.fr/kuka-lwr/kuka-lwr-doc/wikis/home).

If you need the mobile base, turn the key to the right for a few seconds.

How to run the bridge
=====================

Connect to the LIRMM wired network and then open an ssh connection with the robot:
```bash
ssh mcrtc@bazar-rob # password is mcrtc
```

Once connected, just use the `start_bridge` function. You can pass `--help` to get the list of parameters or simply call it as-is to use the default ones. If you need the mobile base, pass `--enable_mobile_base 1`.

The license that applies to the whole package content is **BSD**. Please look at the license.txt file at the root of this repository.

Installation and Usage
=======================

The detailed procedures for installing the mc_rtc-bazar-bridge package and for using its components is based on the [PID](http://pid.lirmm.net/pid-framework/pages/install.html) build and deployment system called PID. Just follow and read the links to understand how to install, use and call its API and/or applications.

For a quick installation:

## Installing the project into an existing PID workspace

To get last version :
 ```
cd <path to pid workspace>/pid
make deploy package=mc_rtc-bazar-bridge
```

To get a specific version of the package :
 ```
cd <path to pid workspace>/pid
make deploy package=mc_rtc-bazar-bridge version=<version number>
```

## Standalone install
 ```
git clone git@gite.lirmm.fr:cobot/mc_rtc-bazar-bridge.git
cd mc_rtc-bazar-bridge
```

Then run the adequate install script depending on your system. For instance on linux:
```
sh share/install/standalone_install.sh
```

The pkg-config tool can be used to get all links and compilation flags for the libraries defined inthe project. To let pkg-config know these libraries, read the last output of the install_script and apply the given command. It consists in setting the PKG_CONFIG_PATH, for instance on linux do:
```
export PKG_CONFIG_PATH=<path to mc_rtc-bazar-bridge>/binaries/pid-workspace/share/pkgconfig:$PKG_CONFIG_PATH
```

Then, to get compilation flags run:

```
pkg-config --static --cflags mc_rtc-bazar-bridge_<name of library>
```

To get linker flags run:

```
pkg-config --static --libs mc_rtc-bazar-bridge_<name of library>
```


About authors
=====================

mc_rtc-bazar-bridge has been developped by following authors: 
+ Benjamin Navarro ()

Please contact Benjamin Navarro -  for more information or questions.



