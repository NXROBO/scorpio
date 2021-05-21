#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

#=================================================
#	System Required: Ubuntu 16.04+
#	Description: Install ROS And Scorpiooooooooooooooooooooooooo
#	Version: 1.0.0
#	Site: http://www.nxrobo.com/
#	scorpio技术讨论与反馈群：8346256
#=================================================


sh_ver="1.1.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"

Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)

PROJECTPATH=$(cd `dirname $0`; pwd)
CAR_COLOR=$(cat /opt/scorpio.txt)
echo ${CAR_COLOR}
if [[ "${CAR_COLOR}" == "silver" ]]; then
	RED_CAR="false"
else
	RED_CAR="true"
fi

#检查系统要求
check_sys(){
        if [[ "${Version}" == "14.04" ]]; then
                ROS_Ver="indigo"
        elif [[ "${Version}" == "16.04" ]]; then
                ROS_Ver="kinetic"
        elif [[ "${Version}" == "18.04" ]]; then
                ROS_Ver="melodic"
        else
                echo -e "${Error} scorpio不支持当前系统 ${OSDescription} !" && exit 1
        fi

}


#安装ROS完整版
install_ros_full(){
		sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654		
		sudo apt-get update
		sudo apt-get install -y ros-${ROS_Ver}-desktop-full
		sudo rosdep init
		rosdep update
		echo "source /opt/ros/${ROS_Ver}/setup.bash" >> ~/.bashrc
		source /opt/ros/${ROS_Ver}/setup.bash
		sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
}

#检测是否需要安装完整版
check_install_ros_full(){
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 检测到当前系统已安装了ROS的${ROSVER}版本!" 
			echo && stty erase ^? && read -p "请选择是否继续安装？ y/n：" choose
			if [[ "${choose}" == "y" ]]; then
				echo -e "${Info}准备安装ROS系统！" 
			else
				exit
			fi
		fi
	fi
	install_ros_full 
}

#安装scorpio依赖库
install_scorpio_require(){
	echo -e "${Info} 准备安装scorpio相关驱动……"
	echo -e "${Info} 设置udev规则……"
	BASEPATH=$(cd `dirname $0`; pwd)
	sudo cp $BASEPATH/src/scorpio_driver/camera/ros_astra_camera/56-orbbec-usb.rules  /etc/udev/rules.d
	sudo $BASEPATH/src/scorpio_driver/lidar/ydlidar_g2/startup/initenv.sh


	service udev reload
	sleep 2
	service udev restart


	sudo udevadm trigger

	echo -e "${Info} 安装所需要的依赖库……"
	#sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
	echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
	sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE

	sudo apt-get update
	sudo apt-get install -y ros-${ROS_Ver}-ecl ros-${ROS_Ver}-ecl-threads ros-${ROS_Ver}-rgbd-launch 
	sudo apt-get install -y ros-${ROS_Ver}-image-common
	sudo apt-get install -y ros-${ROS_Ver}-move-base-* 
	sudo apt-get install -y ros-${ROS_Ver}-serial
	sudo apt-get install -y python-pip python-sklearn libudev-dev
	sudo apt-get install -y ros-${ROS_Ver}-depthimage-to-laserscan ros-${ROS_Ver}-map-server ros-${ROS_Ver}-amcl ros-${ROS_Ver}-gmapping ros-${ROS_Ver}-navigation ros-${ROS_Ver}-navigation-stage ros-${ROS_Ver}-navigation-layers ros-${ROS_Ver}-navigation-tutorials
	sudo apt-get install -y ros-${ROS_Ver}-hector-mapping
	sudo apt-get install -y ros-${ROS_Ver}-frontier-exploration 
	sudo apt-get install -y ros-${ROS_Ver}-rtabmap-ros 
	sudo apt-get install -y ros-${ROS_Ver}-slam-karto
	sudo apt-get install -y ros-${ROS_Ver}-ddynamic-reconfigure  ros-${ROS_Ver}-mrpt-slam ros-${ROS_Ver}-mrpt-icp-slam-2d ros-${ROS_Ver}-ackermann-msgs ros-${ROS_Ver}-robot-localization ros-${ROS_Ver}-teb-local-planner

	sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg 

	pip3.5 install pyinstaller

	echo -e "${Info} 依赖库安装成功……"
}


#编译scorpio
install_scorpio(){
	if [[ "${Version}" == "18.04" ]]; then
		git checkout melodic-devel
	fi
	source /opt/ros/${ROS_Ver}/setup.bash
	catkin_make
	#catkin_make install
}


#完全安装
install_all(){
	check_install_ros_full
	install_scorpio_require
	install_scorpio
}

#远程设置
master_uri_setup(){
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep inet|awk '{print $2}'|awk -F: '{print $2}'`
	if [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	local_ip=$wlp1s_ip
	else 
		echo -e "${Info}未找到有效网络wlp2s0" 
	  	local_ip="127.0.0.1"		
	fi
	export ROS_HOSTNAME=$local_ip
	export ROS_MASTER_URI="http://${local_ip}:11311"
	echo -e "${Info}Using ROS MASTER at ${Red_font_prefix}$ROS_MASTER_URI${Font_color_suffix} from ${Red_font_prefix}$ROS_HOSTNAME${Font_color_suffix}"
}

#让SCORPIO动起来
let_robot_go(){
	echo -e "${Info}" 
	echo -e "${Info}      让SCORPIO动起来" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}    q左前  w前进  e右前    "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}    z左后  s后退  c右后     " 
	echo -e "${Info}                           " 
	echo -e "${Info}    退出请输入：Ctrl + c    " 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch scorpio_teleop app_op.launch model_red:=${RED_CAR}
}


scorpio_test_mode(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash	
	echo -e "${Info}老化测试程序，请将悬挂起来，轮子不要着地进行测试，请选择：
	1.执行5分钟的检测功能；
        2.执行老化检测功能，直至断电。"
        echo -e "${Info}退出请输入：Ctrl + c " 
	echo && stty erase ^? && read -p "请选择 1 或 2 ：" chnum
 	case "$chnum" in
		1)
		roslaunch scorpio_test all_run_test_st.launch model_red:=${RED_CAR}	
		;;
		2)
		roslaunch scorpio_test all_run_test.launch model_red:=${RED_CAR}	
		;;
		*)
		echo -e "${Error} 退出!"	
		;;
	esac
}

#远程（手机APP）控制scorpio
remote_control_robot(){
	master_uri_setup
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}                  " 
	echo -e "${Info} 远程（手机APP）控制scorpio" 
	echo -e "${Info}" 
	echo -e "${Info}远程控制的APP地址:${Yellow_font_prefix}https://raw.githubusercontent.com/iamzhuang/RobotCA/kinetic/Release/control_app-debug.apk"${Font_color_suffix}
	echo -e "${Info}下载安装完成后，打开app，设置Master URI:${Red_font_prefix}http://${local_ip}:11311${Font_color_suffix}" 
	echo -e "${Info}接着就可以开始远程控制SCORPIO了" 
	echo -e "${Info}退出请输入：Ctrl + c" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch scorpio_teleop app_op_remote.launch model_red:=${RED_CAR}
}

#让scorpio跟着你走
people_follow(){
	echo -e "${Info}                  " 
	echo -e "${Info}让scorpio跟着你走" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}                  " 
	echo -e "${Info}请站在scorpio的正前方，与scorpio保持一米左右的距离，然后走动"
	echo -e "${Info}                  " 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch scorpio_follower bringup.launch model_red:=${RED_CAR}
}


#让scorpio使用激光雷达进行导航
scorpio_navigation_2d(){
	echo -e "${Info}" 
	echo -e "${Info}让scorpio使用激光雷达进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}       A.激光雷达已上电连接"
	echo -e "${Info}       B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
	echo -e "${Info}       C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，SCORPIO将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase '^H' && read -p "按回车键（Enter）开始：" 

	roslaunch scorpio_navigation scorpio_navigation.launch model_red:=${RED_CAR} 
}
#让scorpio使用深度摄像头进行导航
scorpio_navigation_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让scorpio使用深度摄像头进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash
	echo -e "${Info}请选择导航方式："
	echo -e "${Info}1.使用2D地图"
	echo -e "${Info}2.使用rtab_map地图"
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" slamnum

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}A.摄像头已连接"
	case "$slamnum" in
		1)
		SLAMTYPE="2d"
		echo -e "${Info}B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
		;;
		2)
		SLAMTYPE="rtab_map"
		echo -e "${Info}B.把机器人放到原来建图的原点。导航正常启动后，如需查看原来建立的３Ｄ地图，点击rviz的Display->Rtabmap cloud->Download map加载３Ｄ地图。"
		;;
		*)
		echo -e "${Error} 错误，默认使用2D地图"
		SLAMTYPE="2d"
		echo -e "${Info}B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
		;;
	esac

	echo -e "${Info}C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，SCORPIO将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	if [[ "${SLAMTYPE}" == "2d" ]]; then
		roslaunch scorpio_navigation scorpio_navigation_camera.launch model_red:=${RED_CAR}
	else
		roslaunch scorpio_rtabmap scorpio_rtabmap_nav.launch model_red:=${RED_CAR}
	fi	
}

#让scorpio使用激光雷达绘制地图(gmapping)
scorpio_build_map_2d(){
	echo -e "${Info}" 
	echo -e "${Info}让scorpio使用激光雷达绘制地图" 
	echo -e "${Info}" 
	echo -e "${Info}请选择SLAM的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} gmapping
	  ${Green_font_prefix}2.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 ：" slamnum
	case "$slamnum" in
		1)
		SLAMTYPE="gmapping"
		;;
		*)
		echo -e "${Error} 错误，默认使用gmapping"
		SLAMTYPE="gmapping"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}           w前进           "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}           s后退           " 
	echo -e "${Info}                           " 
	echo -e "${Info}退出请输入：Ctrl + c        " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	roslaunch scorpio_slam 2d_slam_teleop.launch slam_methods_tel:=${SLAMTYPE} model_red:=${RED_CAR} 
	
}
qrcode_transfer_files(){
	echo -e "${Info}" 
	echo -e "${Info}通过局域网收发文件" 
	echo -e "${Info}" 
	echo -e "${Info}请选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 发送文件（文件名，带上文件绝对路径）
	  ${Green_font_prefix}2.${Font_color_suffix} 接收文件（默认存放在~/Downloads路径中）
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" cnum
	case "$cnum" in
		1)
		echo -e "${Info}请输入文件名，带上文件绝对路径，如 /home/scorpio/a.jpg：
		 退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入要发送的文件：" s_file
		if [ -f "$s_file" ]; then
			echo -e "${Info}本机即将发送文件：${Green_font_prefix}"$s_file"${Font_color_suffix}，请接收端扫码或者直接输入下面的网址接收文件"
		else 
			echo -e "${Info}请输入带绝对路径的文件名"
			exit
		fi
		
		qrcp send $s_file
		;;
		2)
		echo -e "${Info}请输入接收到的文件存放的路径，默认为 /home/scorpio/Downloads：
		退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入文件存放的文件夹路径：" s_file
		if [ -d "$s_file" ]; then
			echo ""
		else 
			echo -e "${Info}${Red_font_prefix}文件夹不存在，将存放在默认文件夹/home/scorpio/Downloads中${Font_color_suffix}"
			s_file="/home/scorpio/Downloads"
		fi
		echo -e "${Info}接收的文件将存放在：${Green_font_prefix}"$s_file"${Font_color_suffix}，目录下，请发送端扫码或者直接输入下面的网址选择文件发送"
		qrcp receive --output=$s_file
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

#让scorpio使用深度摄像头绘制地图
scorpio_build_map_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让scorpio使用深度摄像头绘制地图" 
	echo -e "${Info}" 
	echo -e "${Info}请选择SLAM的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} gmapping
	  ${Green_font_prefix}2.${Font_color_suffix} rtab_map 3D建图
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" slamnum
	case "$slamnum" in
		1)
		SLAMTYPE="gmapping"
		;;
		2)
		SLAMTYPE="rtab_map"
		;;
		*)
		echo -e "${Error} 错误，默认使用gmapping"
		SLAMTYPE="gmapping"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}           w前进           "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}           s后退           " 
	echo -e "${Info}                           " 
	echo -e "${Info}退出请输入：Ctrl + c        " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	if [[ "${SLAMTYPE}" == "rtab_map" ]]; then
		echo -e "${Tip}" 
		echo -e "${Tip}现在使用rtab_map建图，将会删除之前保存的地图，选择‘y’继续建图，其它键直接退出。" 
		echo -e "${Tip}" 
		echo && stty erase ^? && read -p "请选择是否继续y/n：" choose
		if [[ "${choose}" == "y" ]]; then
                	roslaunch scorpio_rtabmap scorpio_rtabmap_teleop.launch model_red:=${RED_CAR}
		else
			return
		fi
        else
		roslaunch scorpio_slam depth_slam_teleop.launch slam_methods_tel:=${SLAMTYPE} model_red:=${RED_CAR}
	fi
	
}

clear_user_data(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/devel/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    清除用户数据"
	echo -e "${Info}将会删掉地图数据，ROS的log记录,和浏览器的所有数据"

	echo && stty erase ^? && read -p "请选择是否继续y/n：" choose

	if [[ "${choose}" == "y" ]]; then
                rm -fr ${PROJECTPATH}/map*
		rm -fr ~/.ros/*
		rm -fr ~/.mozilla/firefox/*
		rosclean purge -y
		echo -e "${Info}    已清除用户数据"
	else
		return
	fi

}
coming_soon(){
	echo -e "${Tip} coming_soon!" 
}


#printf
menu_status(){
	echo -e "${Tip} 当前系统版本 ${OSDescription} !" 
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 当前ROS版本 ${ROSVER} !"
			return
		fi 
	fi
	echo -e "${Error} 未检测到ROS版本，请先安装ROS！可以选择102直接安装。" 
}

tell_us(){
	echo -e ""
	echo -e "${Tip} ------------分隔线--------------" 
	echo -e "${Tip} 网址：www.nxrobo.com" 
	echo -e "${Tip} scorpio技术讨论与反馈QQ群：8346256" 
	echo -e "${Tip} ------------分隔线--------------"
	echo -e ""
}
check_sys
echo -e "--------------分隔线----------------
  scorpio 一键安装管理脚本 ${Red_font_prefix}[v${sh_ver}]${Font_color_suffix}
  请根据右侧的功能说明选择相应的序号。
  注意：101～103为相关环境的安装与设置，
  如果已执行过，不要再重复执行。
--------------分隔线----------------
  ${Green_font_prefix}  0.${Font_color_suffix} 单独编译scorpio
————————————
  ${Green_font_prefix}  1.${Font_color_suffix} 让机器人动起来
  ${Green_font_prefix}  2.${Font_color_suffix} 远程（手机APP）控制scorpio
  ${Green_font_prefix}  3.${Font_color_suffix} 让scorpio跟着你走
  ${Green_font_prefix}  4.${Font_color_suffix} 让scorpio使用激光雷达绘制地图
  ${Green_font_prefix}  5.${Font_color_suffix} 让scorpio使用深度摄像头绘制地图
  ${Green_font_prefix}  6.${Font_color_suffix} 让scorpio使用激光雷达进行导航
  ${Green_font_prefix}  7.${Font_color_suffix} 让scorpio使用深度摄像头进行导航
————————————
  ${Green_font_prefix}100.${Font_color_suffix} 问题反馈
  ${Green_font_prefix}101.${Font_color_suffix} 完整安装
  ${Green_font_prefix}102.${Font_color_suffix} 单独安装ROS环境
  ${Green_font_prefix}103.${Font_color_suffix} 单独安装scorpio依赖
  ${Green_font_prefix}104.${Font_color_suffix} 文件传输
  ${Green_font_prefix}105.${Font_color_suffix} 清除用户数据
 "
menu_status
echo && stty erase ^? && read -p "请输入数字：" num
case "$num" in
	0)
	install_scorpio
	;;
	1)
	let_robot_go
	;;
	2)
	remote_control_robot
	;;
	3)
	people_follow
	;;
	4)
	scorpio_build_map_2d
	;;
	5)
	scorpio_build_map_3d
	;;
	6)
	scorpio_navigation_2d
	;;
	7)
	scorpio_navigation_3d
	;;
	98)
	scorpio_test_mode
	;;
	100)
	tell_us
	;;
	101)
	install_all
	;;
	102)
	check_install_ros_full
	;;
	103)
	install_scorpio_require
	;;
	104)
	qrcode_transfer_files
	;;
	105)
	clear_user_data
	;;	
	*)
	echo -e "${Error} 请输入正确的数字 "
	;;
esac
