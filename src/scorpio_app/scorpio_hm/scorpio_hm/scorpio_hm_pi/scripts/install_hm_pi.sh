#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"

echo '安装spark_hm_pi的340串口'
string=$(ls /dev/ttyUSB*) 
array=(${string// / })  
echo -e "查找到如下USB串口设备："
echo -e "${Green_font_prefix}${string}${Font_color_suffix}"
Counter=$(lsusb | grep QinHeng | wc -l)
if [[ "${Counter}" > 1 ]]; then
	echo  -e "检测到多个CH340串口"
else
	echo -e "${Error} 只检测到1个CH340串口" && exit 1
fi
rm -fr /tmp/spark_device.txt
setcounter=0
for var in ${array[@]}
do
	TXT_TXT=$(udevadm info -a -n $var | grep -B 2 "DRIVERS==\"ch341\"" | grep -B 1 KERNELS |awk -F "[\"\"]" '{print $2}')
	if [ -n "${TXT_TXT}" ]; then
		echo -e "当前串口${Red_font_prefix}$var${Font_color_suffix} 是哪个设备的?请选择:"
		echo -e "${Red_font_prefix}1${Font_color_suffix}.底盘；"
		echo -e "${Red_font_prefix}2${Font_color_suffix}.星火派开发板;"
		echo -e "${Red_font_prefix}3${Font_color_suffix}.None"
		echo -e "${Red_font_prefix}4${Font_color_suffix}.恢复到无星火派开发板的状态"
		echo && stty erase ^? && read -p "请输入对应序号（回车确定）：" chnum
		case "$chnum" in
			1)
			echo -e "你选择 1.底盘的${Red_font_prefix}KERNELS=${TXT_TXT}${Font_color_suffix}"	

			echo "KERNELS==\"${TXT_TXT}\",MODE:=\"0666\",GROUP:=\"dialout\",SYMLINK+=\"CanBase\"" >> /tmp/spark_device.txt	
			((setcounter++))
			;;
			2)
			echo -e "你选择 2.星火派开发板${Red_font_prefix}KERNELS=${TXT_TXT}${Font_color_suffix}"	
			echo "KERNELS==\"${TXT_TXT}\",MODE:=\"0666\",GROUP:=\"dialout\",SYMLINK+=\"SPARK-HM-PI\"" >> /tmp/spark_device.txt
			((setcounter++))
			;;
			4)
			echo -e "恢复到无星火派开发板的状态"	
			echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="CanBase", MODE:="0666",OWNER:="root"' > /tmp/spark_device.txt
			setcounter=3
			break
			;;			
			3)
			echo -e "None"
			;;		
			*)
			echo -e "错误，退出"
			break
			;;
		esac
		if [[ ${setcounter} -ge 3 ]]; then
			break
		fi
	fi
	echo  "${TXT_TXT}" 
	echo $var
done 
if [[ ${setcounter} -ge 2 ]]; then
	echo -e "确定执行以上更改？"
	echo && stty erase ^? && read -p "确定执行以上更改？ y/n：" choose
	if [[ "${choose}" == "y" ]]; then
		#sudo mv /tmp/spark_device.txt  /etc/udev/rules.d/spark-usb-serial.rules
		sudo mv /tmp/spark_device.txt  /etc/udev/rules.d/can_serial.rules
		sudo udevadm trigger
	else
		echo -e "${Error} 退出不执行" && exit 1
	fi
else
	echo -e "${Error} 退出不执行" && exit 1
fi
echo '安装完成'	
exit	
	

