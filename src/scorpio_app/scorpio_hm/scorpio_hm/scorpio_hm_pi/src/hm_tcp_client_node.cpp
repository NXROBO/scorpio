#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
using namespace std;

#define MAXLINE 4096
#include "../../common/include/common/json/json.h"
#define WIFIFILENAME "/home/spark/wifi_info.txt"
int sockfd = -1;
std::string get_file_content(std::string filename)
{
    std::fstream m_fs;
    m_fs.open(filename.c_str(), ios::in);
    if (!m_fs.is_open())
    {
        cout << "读取文件失败" << endl;

    }
    string buf;
    string str_buf;
    while (getline(m_fs,buf))
    {
        cout << buf << endl;
        str_buf = str_buf+buf;
    }
    printf("msg:%s\n",str_buf.c_str());
    return str_buf;

}

std::string parse_ipaddress(std::string filename)
{
    std::string ipaddress;
    Json::Value _json;
    Json::Reader reader_js(Json::Features::strictMode());
    std::string input_msg = get_file_content(filename);
    if(reader_js.parse (input_msg, _json))
    {
        ipaddress = _json["wifi_slaver"]["ipaddress"].asString();
        ROS_INFO("the slave spark-hm-pi ip address is %s", ipaddress.c_str());
    }
    else
    {
        ROS_ERROR("%s is not a valid json file", WIFIFILENAME);
        ROS_ERROR("please check json format validity in wifi_info.txt");
        ROS_ERROR("or use script/reset_wifi_info_file.sh to reset the file and re-config it");
    }

    return ipaddress;
}


int send_tcp_msg(std::string server_ip, std::string msg)
{
    int   n;
    char  recvline[4096], sendline[4096];
    struct sockaddr_in  servaddr;
    while(1)
    {
        if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
            return 0;
        }

        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(10006);
        if(inet_pton(AF_INET, server_ip.c_str(), &servaddr.sin_addr) <= 0)
        {
            printf("inet_pton error for %s\n",server_ip.c_str());
            close(sockfd);
            continue;
        }

        if(connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
        {
            printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
            close(sockfd);
            continue;
        }
        else
        {
            printf("connect to server successfully!!!!\n");
            break;
        }

    }
    if(send(sockfd, msg.c_str(), strlen(msg.c_str())+1, 0) < 0)
    {
        printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
    }
    close(sockfd);

    return 0;
}

//判断是否转发HMPI的显示命令
void hm_gate_cb(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("sending the message to slaver hmpi:%s", msg->data.c_str());
    send_tcp_msg(parse_ipaddress(WIFIFILENAME),msg->data);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hm_tcp_client_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<std_msgs::String>("/hm_task_gate_cmd", 1, hm_gate_cb);

    ros::spin();
    return 0;
}
