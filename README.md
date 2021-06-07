# ccri_cabinet

CCRI Pick and Place Cabinet Object Detection

Utilizes photoresistors to indicate whether or not an object is covering one of the fiducial markers.
External lighting may need to be added to cabinet to ensure proper operation (LED strips, etc.) Publishes 
data over ROS Serial and data can be stored locally on the PC. Requires you to run the command: 

rosrun roserial_python serial_node.py /dev/ttyACM0   //where /dev/ttyACM0 is the port the uController is on 

Tested on Teensy 3.6 and 3.2 with teensyduino, results may vary on other boards.Tested on Ubuntu 16.04 with
ROS Kinetic. 


