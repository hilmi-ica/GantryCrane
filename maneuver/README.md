# Tutorials
### Models
Please download these depth estimation models and put them on the "models" folder:
- [DPT-Large](https://github.com/intel-isl/DPT/releases/download/1_0/dpt_large-midas-2f21e586.pt)
- [DPT-Hybrid](https://github.com/intel-isl/DPT/releases/download/1_0/dpt_hybrid-midas-501f0c75.pt)
- [MiDaS v2.1 Small](https://github.com/AlexeyAB/MiDaS/releases/download/midas_dpt/midas_v21_small-70d6b9c8.pt)
maneuvers.py

### Setup
- Run roscormaneuvers.py -t reshuffle -x a -y be on terminal
> roscore
- Run ros serial python on terminal
> rosrun rosserial_python serial_node.py /dev/ttyUSB0
- Run rqt on terminal
> rqt

### Calibration
- Run control1.py script
- Change mode in rqt to 1

### Manuever
These are the steps to operate crane  manuevers:
- Run rcv2.py script with the selected detection models (my_ssd_mobnet for multicolor and SSDMobileNet_Container_160821 for blue color container)
- Check the encoder value (around at the far left and around 28000 at the far right)
- If the encoder value is not right, do a calibration
- Run maneuvers.py script on terminal with certain command for certain type of maneuver:
  1. Lift-off:
  > python maneuvers.py -t lift-off
  2. Lift-on:
  > python maneuvers.py -t lift-on -x a -y b
  3. Reshuffle:
  > python maneuvers.py -t reshuffle -x a -y b
  Note: a and b are number between 0-5 and 0-3 for row and column location of related container respectively from left to right and down to up
- Change mode in rqt (8, 9, and 10 for lift-off, lift-on, and reshuffle maneuver respectively)
