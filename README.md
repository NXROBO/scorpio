# NXROBO scorpio
<img src="https://raw.githubusercontent.com/NXROBO/scorpio/master/src/scorpio/scorpio_description/pic/scorpio.jpg" width="600">

## 说明 Description
 
- 本说明为初学者体验版，[这里](https://github.com/NXROBO/scorpio/blob/master/README_Detailed.md)有详细说明的版本。

## 列表 Table of Contents

* [功能包说明packages-overview](#功能包说明packages-overview)
* [使用usage](#使用usage)

## 功能包说明packages-overview

* ***src*** : scorpio的源代码，包括底层配置，硬件驱动，和各个应用功能包等。
* ***doc*** : 软硬件依赖包。

## 使用usage

### 系统要求 Prequirement

* System:	Ubuntu 14.04+
* ROS Version:	indigo or kinetic(Desktop-Full Install) 

### 下载安装 Download and install

* 下载工作空间 Download the workspace:
```yaml
git clone https://github.com/NXROBO/scorpio.git
```
* 安装依赖库 Install libraries and dependencies:
```yaml
cd scorpio
./onekey.sh
```
* 根据提示选择103 Choose NO.103
```yaml
103
```
### 编译运行 compile and run
```yaml
catkin_make
```
* 如果编译一切正常，可根据提示运行相关例程。If everything goes fine, test the examples as follow:
```yaml
./onekey.sh
```

