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
* rosrun bluesea2 bluesea2_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=500000 _firmware_version:=2 _output_scan:=true _output_cloud:=true _with_resample:=true _resample_res:=0.5 _unit_is_mm:=true _with_confidence:=true
* or use roslaunch src/bluesea/launch/LDS-50C-2.launch
    
## if your lidar model is LDS-15BDM or LDS-25BDM:
* rosrun bluesea2 bluesea2_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2 _output_scan:=true _output_cloud:=true _unit_is_mm:=false _with_confidence:=true _raw_bytes:=2
* or use roslaunch src/bluesea2/launch/LDS-15BDM.launch    

3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 

How to start/stop LiDAR detection 
=====================================================================
1) resume detection : rosservice call /your_node/start_motor
2) stop detection : rosservice call /your_node/stop_motor

How to run Lanhai ros node (UDP Network Version)
=====================================================================
1) sudo ifconfig eth0:1 192.168.158.200 # add sub net
2) rosrun bluesea2 bluesea2_node _frame_id:=map _type:=udp _lidar_ip:=192.168.158.91 _firmware_version:=2
3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 

## if your lidar model is LDS-50C-E :
* use roslaunch src/bluesea/launch/LDS-50C-E.launch


Parameters
=====================================================================
* std::string type; // LiDAR comm type, could be "uart", "tcp" or "udp"
* std::string platform; // LiDAR hardware platform
* std::string dump;	// file path of dump raw data, for debug

// for serial port comm
* std::string port; // serial port device path
* int baud_rate; // baud rate, -1 : auto detect current baud rate

// for network comm
* std::string lidar_ip; // LiDAR's network address 
* std::string group_ip; // multicast address
* int lidar_port; // lidar's port (TCP / UDP)
* int local_port; // ROS machine's port (TCP / UDP)

// for intput data format
* bool unit_is_mm; //  true : unit of raw data distance is CM, false: MM
* bool with_confidence; // true: raw data with intensity, false: no intensity
* bool with_checksum; // true : enable packet checksum

// output data type
* bool output_scan; // true: enable output angle+distance mode, false: disable
* bool output_cloud; // true: enable output xyz format data, false : disable
* bool output_360; // true: collect multiple RawData packets (360 degree), then publish
				// false: publish every RawData (36 degree)
* std::string frame_id;	// frame information, could be used for rviz
* bool from_zero; // true : angle range [0 - 360), false: angle range [-180, 180)

// is lidar inverted
* bool inverted; // inverted installed
* bool reversed; // data's angle increment

// angle composate
* bool with_resample; // resample angle resolution
* double resample_res; // 0.5: resample angle resolution @ 0.5 degree 


// output data format
* int normal_size; // abnormal packet (points number < normal_size) will be droped

// angle filter
* bool with_angle_filter ; // true: enable angle filter, false: diable
* double min_angle; // angle filter's low threshold, default value: -pi
* double max_angle; // angle filters' up threashold, default value: pi

* double max_dist;


Dynamic Reconfigure Parameters
=====================================================================
int rpm; // motor's scaning RPM [300, 1500]

command line like this:
rosrun dynamic_reconfigure dynparam set /lidar1/lidar01 "{'rpm':700}"


How to control Lanhai ros node  start  and stop
=====================================================================
* client:      
 			
												   arg1  state    arg2:choose lidar serial number

			start  or stop  one lidar
			rosrun bluesea2  bluesea2_node_client  start/stop     0/1/2/... 
			start or stop   all lidar
			rosrun bluesea2  bluesea2_node_client  start/stop     -1          
	        

* server:     

 			roslaunch bluesea2  xxx.launch



How to enable multiple radars and use only one port
=====================================================================
refer to  dual-LDS-50C-C30E.launch
warming: The following parameters value is must different

	lidar_ip/lidar*_ip  

	lidar_port/lidar*_port

	topic/topic*   
