# bluesea
ROS driver for Lanhai USB 2D LiDAR 

How to build Lanhai ros driver
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build 

How to run Lanhai ros node (Serial Port Version)
=====================================================================
1) Copy UDEV rule file : sudo cp src/LHLiDAR.rules /etc/udev/rules.d/
2) or Run : sudo chmod 666 /dev/ttyUSB0 # make usb serial port readable

## if your lidar model is LDS-50C-2 :
* rosrun bluesea bluesea_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=500000 _firmware_version:=2 _output_scan:=true _output_cloud:=true _with_resample:=true _resample_res:=0.5 _unit_is_mm:=true _with_confidence:=true
* or use roslaunch src/bluesea/launch/LDS-50C-2.launch
    
## if your lidar model is LDS-15BDM or LDS-25BDM:
* rosrun bluesea bluesea_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2 _output_scan:=true _output_cloud:=true _unit_is_mm:=false _with_confidence:=true _raw_bytes:=2
* or use roslaunch src/bluesea/launch/LDS-15BDM.launch    

3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 

How to start/stop LiDAR detection 
=====================================================================
1) resume detection : rosservice call /your_node/start_motor
2) stop detection : rosservice call /your_node/stop_motor

How to run Lanhai ros node (UDP Network Version)
=====================================================================
1) sudo ifconfig eth0:1 192.168.158.200 # add sub net
2) rosrun bluesea bluesea_node _frame_id:=map _type:=udp _dev_ip:=192.168.158.91 _firmware_version:=2
3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 



Parameters
=====================================================================
* std::string type; // LiDAR comm type, could be "uart" or "udp"
* std::string dump;	// file path of dump raw data, for debug

// for serial port comm
* std::string port; // serial port device path
* int baud_rate; // baud rate, -1 : auto detect current baud rate

// for network comm
* std::string dev_ip; // network 
* int udp_port, tcp_port; 

// for intput data format
* bool unit_is_mm; //  0 : unit of raw data distance is CM, 1: MM
* bool with_confidence; // 1: raw data with intensity, 0: no intensity
* bool with_checksum; // 1 : enable packet checksum

// output data type
* bool output_scan; // 1: enable output angle+distance mode, 0: disable
* bool output_cloud; // 1: enable output xyz format data, 0 : disable
* bool output_360; // 1: collect multiple RawData packets (360 degree), then publish
				// 0: publish every RawData (36 degree)
* std::string frame_id;	// frame information, could be used for rviz

// angle composate
* bool with_resample; // resample angle resolution
* double resample_res; // 0.5: resample angle resolution @ 0.5 degree 


// output data format
* int normal_size; // abnormal packet (points number < normal_size) will be droped

// angle filter
* int with_angle_filter ; // 1: enable angle filter, 0: diable
* double min_angle; // angle filter's low threshold, default value: -pi
* double max_angle; // angle filters' up threashold, default value: pi

* double max_dist;

Dynamic Reconfigure Parameters
=====================================================================
int rpm; // motor's scaning RPM [300, 1500]

command line like this:
rosrun dynamic_reconfigure dynparam set /lidar1/lidar01 "{'rpm':700}"


