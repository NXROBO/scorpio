#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"

echo '检查spark_hm_pi的340串口'



setcounter=0
while [[ $choice != q ]]; do
	string=$(ls /dev/ttyUSB*) 
	array=(${string// / })  
	mycounter=0
	for var in ${array[@]}
	do
		TXT_TXT=$(udevadm info -a -n $var | grep -B 2 "DRIVERS==\"ch341\"" | grep -B 1 KERNELS |awk -F "[\"\"]" '{print $2}')
		if [ -n "${TXT_TXT}" ]; then
			usbtty[mycounter]="${var}的KERNELS${TXT_TXT}   "
			((mycounter++))
		fi
		#echo "$mycounter $var 的号为：${TXT_TXT} " 

	done 
	#组成空格
	stringZ="340 USB设备:${usbtty[*]}"
	if [[ ${#stringZ} -ge ${setcounter} ]]; then
		spacenull=" "
		kg=" "	
		for svar in 50
		do 
			spacenull=$spacenull$kg
		done 
		setcounter=${#stringZ}
	fi

	echo -ne ${spacenull} 
	echo -ne "${Yellow_font_prefix}340 USB设备${Font_color_suffix}: ${Red_font_prefix}${usbtty[*]}${Font_color_suffix} \r"
	for ii in mycounter
	do
		usbtty[ii]=""
	done 
done
