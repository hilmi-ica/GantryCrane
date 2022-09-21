# Tutorials
### Setup
- Run roscore on terminal
```
roscore
```
- Run ros serial python on terminal
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
- Run rqt on terminal
```
rqt
```

### Calibration
- Run control1.py script
- Change mode in rqt to 1

### Step Control
These are the steps to operate crane with step control module:
- Run coba.py script
- Change the control method (step or reference) in Line 369-377 of modular_control_step.py script
- Change the setpoint value in Line 27 (for step) or 240-253 (for reference) of modular_control_step.py script
- Make sure the controller and its gain parameter value is correct (PID-PID or PID-PD controller)
- Check the encoder value (around 1000 at the far left and around 28000 at the far right) by running this command on terminal:
```
rostopic echo encoder
```
- If the encoder value is not right, do a calibration
- Run modular_control_step.py script
- Change mode in rqt to 5

### Scan Control
These are the steps to operate crane with object detection:
- Run detectionros.py script with the selected detection models (my_ssd_mobnet for multicolor and SSDMobileNet_Container_160821 for blue color container)
- Make sure the controller and its gain parameter value is correct (PID-PID or PID-PD controller)
- Check the encoder value (around 1000 at the far left and around 28000 at the far right) by running this command on terminal:
```
rostopic echo encoder
```
- If the encoder value is not right, do a calibration
- Run modular_control_scan.py script
- Change mode in rqt to 2
