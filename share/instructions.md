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