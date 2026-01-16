##ROS driver for COM and TCP

HOW TO USE
=====================================================================
        1) Clone this ros package folder(xpkg_comm) to your catkin's workspace src folder
    	2) Running catkin_make to build 
        3) Add path of setup.bash to ~/.bashrc like: echo "source ~/workspace/devel/setup.bash">> ~/.bashrc	source ~/.bashrc
        4) Copy UDEV rule file(xpkg_comm/scripts/CAN2COM_HUB.rules) to system(/etc/udev/rules.d/)
        5) Or run file(xpkg_comm/scripts/script_init.py) to add UDEV rule
        6) Link the CAN-COM hub
        7) Roslaunch xnode_comm_test.launch(xpkg_comm/launch/xnode_comm_test.launch)

Special note: please use include/LIB_JSON/ArduinoJson.h,view < https://arduinojson.org > for details

PARAMETER
=====================================================================
        dev_list:               Show ROS_INFO of new device in terminal
        com_enable:             Open COM function
        com_channel_common: 	Use raw data of COM(need enable COM)
        com_channel_xstd:       Convert data to XSTD protocol(need enable COM)
        tcp_enable:             Open TCP function
        tcp_channel_common: 	Use raw data of TCP(need enable TCP)
        tcp_channel_xstd:       Convert data to XSTD protocol(need enable TCP)
        tcp_addr:               TCP addresss
        tcp_port:               TCP port

JSON OUT LIST(/xtopic_comm/device_list_json)
=====================================================================
        dev_count:              Count of device
        dev_class[]:            Vector of class data
        dev_type[]:             Vector of type data
        dev_number[]:           Vector of number data
        dev_enable[]:           Vector of enable data
