#pragma once
#pragma comment(lib,"ws2_32.lib")
#define _CRT_SECURE_NO_WARNINGS
#include <WS2tcpip.h>
#include <WinSock2.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <ctime>
#include <sys/utime.h>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <regex>
#include <thread>
#include <atomic>
#include <array>
#include "serialport.h"
#include<math.h>
#include "rsi_lib.h"
#include <fstream>
//#include<aris.hpp>

//using namespace aris::dynamic;

using namespace std;
using namespace tinyxml2;



//定义端口号
const unsigned int Port = 59150;
//定义收发数据缓冲区大小
const int recvBufLen = 1024;
const int sendMsgLen = 1024;

void packXML(double Tech_T2[10], double RKorr[6], std::string &IPOC);
bool extractTimestamp(char * sendBuf, std::string &IPOC);


struct RobotData
{
	double x, y, z, a, b, c;
};
struct rtn_value
{
	double corr_pos[6];
	bool inited_flag;
};
auto cpt_admit(float *fce, double *eul,double *ctr_fce, bool RSI_status)->std::array<double, 6>;

//rtn_value cpt_admit(float *fce, double *eul, double *ctr_fce, bool RSI_status);
int main()
{
	
	//step1:创建上位机PC与KUKA机器人控制器之间的UDP通讯

	/*加载socket库*/
	WORD wVersionRequested;
	WSADATA wsaData;
	wVersionRequested = MAKEWORD(2, 2);
	int err;

	if (WSAStartup(wVersionRequested, &wsaData) != 0) //指定winsock的版本并初始化
	{
		std::cout << "socket库初始化失败。" << std::endl;
		//return -1;
	}
	
	//创建服务器socket
	SOCKET listenSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);//UDP协议
	if (listenSock == INVALID_SOCKET)
	{
		std::cout << "服务器socket创建失败" << std::endl;
	}
	
	//配置服务器监听地址和端口，并绑定一个套接字到一个IP和Port
	SOCKADDR_IN serverAddr;
	serverAddr.sin_family = AF_INET;
	in_addr addr;
	inet_pton(AF_INET, "192.168.1.20", (void *)&addr);//"192.168.1.20"//127.0.0.1
	serverAddr.sin_addr = addr;//IP地址
	serverAddr.sin_port = htons(Port);//端口
	int bindret = ::bind(listenSock, (SOCKADDR *)&serverAddr, sizeof(serverAddr));
	if (bindret == SOCKET_ERROR)
	{
		std::cout << "绑定服务器socket失败！" << std::endl;
		return -1;
	}
	
	//创建连接
	//UDP服务端不需要监听
	SOCKADDR_IN clientAddr;
	int cAdrrLen = sizeof(clientAddr); //必须指定长度，否则会导致accept返回10014错误
	int sAddrLen = sizeof(serverAddr);
	int dataNum = 0;
	char recvBuf[recvBufLen];
	int recvLen;
	auto rsi_info = RSIState();
	

	//声明数据处理线程里用到的变量
	std::atomic<RobotData> robot_data;
	robot_data = RobotData{ 0,0,0,0,0,0 };
	std::atomic_int64_t ipoc = 0;
	std::atomic<RobotData> recv_RobData;
	recv_RobData = RobotData{ 0,0,0,0,0,0 };
	std::atomic_bool rsi_is_connected = false;
	//std::atomic_bool init_is_finished = false;

	//创建数据处理线程，完成力传感器RS422串口数据采集、机器人位姿补偿量计算以及主线程和子线程之间的数据交互
	auto make_data = std::thread([&]() 
	{
		RobotData local_data{0,0,0,0,0,0};
		//bool rsi_is_connected = false;
		std::int64_t ipoc_start = 0;
		bool pre_conn_state = false;

		//创建串口对象并初始化建立连接
		SPcom RS422;		
		bool sp_is_connected = RS422.InitPort(4, 230400, NOPARITY, 8, ONESTOPBIT);
		if (!sp_is_connected)
		{
			std::exception("初始化串口失败");
		}
		else
		{
			std::cout << "初始化串口成功" << std::endl;
			
		}


		

		std::chrono::high_resolution_clock clock;
		auto begin_time = clock.now();
		while (RS422.hComm != INVALID_HANDLE_VALUE)
		{
			//读串口获取力传感器数据
			/*
			unsigned char sendData[4];
			sendData[0] = 0x49;
			sendData[1] = 0xAA;
			sendData[2] = 0x0D;
			sendData[3] = 0x0A;
			RS422.write_char(sendData, 4);
			*/
			//double euler[3];
			RS422.ReadPortThread(RS422.force);
			
			//static int icount1 = 0;
			//if (++icount1 % 100 == 0)
			//{
			//	std::cout << "X_fce：" << RS422.force[0] << "Y_fce：" << RS422.force[1] << std::endl;
			//}
			double euler[6];
			double ctrl_force[6]{ 0,0,-8,0,0,0 };
			//auto correction = cpt_admit(RS422.force, euler);
			//float test0[6]{ 100,100,100,100,100,100 }, test1[6]{200,1000,1000,200,200,200};
			//static int is_begin= 0;
			//auto correction = cpt_admit(is_begin++ < 10000 ? test0 : test1);
			/*auto correction = cpt_admit(RS422.force,);*/
			//static int count_0 = 0;
			//if (count_0++ % 2 == 0)
			//{
			////	std::cout <<"correct:" << std::setw(10) << correction[0] << std::setw(10) << correction[1] << std::setw(10) << correction[2]
			////		<< std::setw(10) << correction[3] << std::setw(10) << correction[4] << std::setw(10) << correction[5] << std::endl;
			//	//printf("Fx= %2f N,Fy= %2f N,Fz= %2f N,Mx= %2f N.M,My= %2f N.M,Mz= %2f N.M\n", RS422.force[0], RS422.force[1], RS422.force[2], RS422.force[3], RS422.force[4], RS422.force[5]);
			//	printf("Fx= %2f ,Fy= %2f ,Fz= %2f ,Mx= %2f ,My= %2f ,Mz= %2f \n", RS422.force[0], RS422.force[1], RS422.force[2], RS422.force[3], RS422.force[4], RS422.force[5]);
			//}

			//判断RSI连接是否成功
			auto time = ipoc.load();
			auto is_connected = rsi_is_connected.load();
			//cpt_admit(RS422.force, euler, is_connected);
			if (time != 0 && pre_conn_state == false && is_connected == true)
			{
				
				ipoc_start = time;
				std::cout << "RSI 连接成功" << std::endl;
				
			}
			else if(pre_conn_state == true && is_connected == false)
			{
				std::cout << "RSI 断开连接" << std::endl;
				cpt_admit(RS422.force, euler,ctrl_force, is_connected);

			}
			pre_conn_state = is_connected;

			if (is_connected)
			{
				///*
				//获取机器人位姿
				auto loc_recvPos = recv_RobData.load();

				//double euler[3];
				euler[0] = loc_recvPos.a;
				euler[1] = loc_recvPos.b;
				euler[2] = loc_recvPos.c;
				euler[3] = loc_recvPos.x;
				euler[4] = loc_recvPos.y;
				euler[5] = loc_recvPos.z;

				auto correction = cpt_admit(RS422.force,euler, ctrl_force,is_connected);
				static int icount = 0;
				//if (++icount % 100 == 0)
				//{
				//	std::cout << "inited_flag：" << correction.inited_flag <<std::endl;
				//}
				//*/
				//local_data.c = (1 - std::cos((time - ipoc_start) / 200.0))*2;
				
				//if (local_data.b <= 4 && local_data.c <= 4)
				//{
					//local_data.c += 0.004;
					//local_data.b += 0.004;
				
					//local_data.c = min(local_data.c, 3.99);
					//local_data.b = min(local_data.b, 2.99);
				//robot_data.store(local_data);

				//static int count = 0;
				//if (count++ % 100 == 0)
				//{
				//	std::cout << "z:" << robot_data.load().z << std::endl;
				//}

				local_data.x = correction[0];
				local_data.y = correction[1];
				local_data.z = correction[2];
				local_data.a = correction[3];
				local_data.b = correction[4];
				local_data.c = correction[5];
				robot_data.store(local_data);
				//auto init_is_over = correction.inited_flag;
				//init_is_finished.store(init_is_over);
				//static int count = 0;
				//if (count++ % 100 == 0)
				//{
					//std::cout << "fx:"<<std::setw(10)<< RS422.force[0] << "  fy:" << std::setw(10) << RS422.force[1] << "  fz:" << std::setw(10) << RS422.force[2]
					//	<< "  Mx:" << std::setw(10) << RS422.force[3] << "  My:" << std::setw(10) << RS422.force[4] << "  Mz:" << std::setw(10) << RS422.force[5]
					//	<< "  dx:" << std::setw(10) << local_data.x << "  dy:" << std::setw(10) << local_data.y << "  dz:" << std::setw(10) << local_data.z 
					//	<< "  dA:" << std::setw(10) << correction[3] << "  dB:" << std::setw(10) << correction[4] << "  dC:" << std::setw(10) << correction[5]
					//	<< std::endl;
					//std::cout << "fx:" << std::setw(10) << RS422.force[0] << "  fy:" << std::setw(10) << RS422.force[1] << "  fz:" << std::setw(10) << RS422.force[2]
					//	<< "  Mx:" << std::setw(10) << RS422.force[3] << "  My:" << std::setw(10) << RS422.force[4] << "  Mz:" << std::setw(10) << RS422.force[5]
					//	<< "  dx:" << std::setw(10) << local_data.x << "  dy:" << std::setw(10) << local_data.y << "  dz:" << std::setw(10) << local_data.z
					//	<< "  dA:" << std::setw(10) << correction[3] << "  dB:" << std::setw(10) << correction[4] << "  dC:" << std::setw(10) << correction[5]
					//	<< std::endl;
				//}
			}


			/*if (std::abs(RS422.force[2]) > 190)
			{
				return 0;
			}
			*/
		/*	static int i = 0;
			std::cout << "test" << ++i << std::endl;

			if (i == 100)
				std::cout << "test" << std::endl;
				*/
			//std::this_thread::sleep_for(std::chrono::milliseconds(1));
			static int count_local_ = 0;
			std::this_thread::sleep_until(begin_time + std::chrono::milliseconds((count_local_++) * 2));
		}
	});


	//UDP读写循环
	while (true)
	{

		std::memset(recvBuf, 0, sizeof(recvBuf));
		recvLen = recvfrom(listenSock, recvBuf, recvBufLen, 0, (SOCKADDR *)&clientAddr, &cAdrrLen); //UDP协议，在客户端发送数据给服务端时自动创建连接
		
		if (recvLen == SOCKET_ERROR)
		{
			std::cout << "接收数据失败！" << std::endl;
			break;
		}
		else if (recvLen == 0)
		{
			break;
		}
		else
		{
			char client_ip[16];
			std::memset(client_ip, 0, 16);
			inet_ntop(AF_INET, &clientAddr.sin_addr, client_ip, 16);
			//std::cout << "接收来自" << client_ip << "的数据" << ++dataNum << ":" << recvBuf << std::endl;
		}
		
		//step2：解析客户端（KUKA机器人）发送过来的XML格式数据
		char *ipocStart = strstr(recvBuf, "<IPOC>");
		char *ipocEnd = strstr(recvBuf, "</IPOC>");
		int ipocLen = ipocEnd - ipocStart - 6;
		char ipocChar[10];
		std::strncpy(ipocChar, ipocStart + 6, ipocLen);

		std::int64_t ipoc_int64 = std::atoll(ipocChar);
		ipoc.store(ipoc_int64);
		ipocChar[ipocLen] = 0;

		///*
		
		rsi_info.get_tag_data(recvBuf);
		
		auto loc_recvPos = RobotData();
		loc_recvPos.x = rsi_info.act_cart_pos[0];
		loc_recvPos.y = rsi_info.act_cart_pos[1];
		loc_recvPos.z = rsi_info.act_cart_pos[2];
		loc_recvPos.a = rsi_info.act_cart_pos[3];
		loc_recvPos.b = rsi_info.act_cart_pos[4];
		loc_recvPos.c = rsi_info.act_cart_pos[5];
		recv_RobData.store(loc_recvPos);
		rsi_is_connected.store(rsi_info.rsi_is_connected);
		//*/
		//char msg1[1024] = "<Sen Type=\"ImFree\">..<EStr>Message from RSI TestServer</EStr>..<Tech T21=\"1.09\" T22=\"2.08\" T23=\"3.07\" T24=\"4.06\" T25=\"5.05\" T26=\"6.04\" T27=\"7.03\" T28=\"8.02\" T29=\"9.01\" T210=\"10.00\" />..<RKorr X=\"%f\" Y=\"%f\" Z=\"%f\" A=\"%f\" B=\"%f\" C=\"%f\" />..<DiO>0</DiO>..<IPOC>%s</IPOC>..</Sen>";
		//auto init_is_over = init_is_finished.load();

		char msg1[1024] = "<Sen Type=\"ImFree\">..<EStr>Message from RSI TestServer</EStr>..<Tech T21=\"1.09\" T22=\"2.08\" T23=\"3.07\" T24=\"4.06\" T25=\"5.05\" T26=\"6.04\" T27=\"7.03\" T28=\"8.02\" T29=\"9.01\" T210=\"10.00\" />..<RKorr X=\"%f\" Y=\"%f\" Z=\"%f\" A=\"%f\" B=\"%f\" C=\"%f\" />..<DiO>125</DiO>..<IPOC>%s</IPOC>..</Sen>";
		char send_msg[1024]{ 0 };

		auto local_data = robot_data.load();
		
		std::sprintf(send_msg, msg1,local_data.x, local_data.y, local_data.z, local_data.a, local_data.b, local_data.c, ipocChar);
		//std::cout << send_msg << std::endl;

		int sendLen;
		sendLen = sendto(listenSock, send_msg, strlen(send_msg), 0, (SOCKADDR *)&clientAddr, cAdrrLen); //UDP协议
		if (sendLen == SOCKET_ERROR)
		{
			std::cout << "发送数据失败！" << std::endl;
			break;
		}
		else if (sendLen == 0)
		{
			break;
		}
		else
		{
			//std::cout << "发送数据成功！" << std::endl;
		}
		
	}

	err = shutdown(listenSock, 2);
	if (err == SOCKET_ERROR)
	{
		std::cout << "shutdown 失败！" << std::endl;
		//return -1;
	}

	//关闭socket
	err = closesocket(listenSock);
	if (err == SOCKET_ERROR)
	{
		std::cout << "关闭socket失败！" << std::endl;
		//return -1;
	}
	else
	{
		std::cout << "socket已关闭！" << std::endl;
	}
	//卸载socket库
	WSACleanup();
	if (WSACleanup() != 0)
	{
		std::cout << "WSACleanup失败！" << std::endl;
		return 0;
	}
	else
	{
		std::cout << "已成功卸载socket库。" << std::endl;
	}
	
	return 0;
}


int parse_kuka_xml(const char * xstr)
{
	tinyxml2::XMLDocument doc;
	doc.Parse(xstr);
	if (doc.Error() == true)
	{
		std::cout << "解析客户端的XML文件出错" << doc.ErrorLineNum() << std::endl;
		return 0;
	}

}

void packXML(double Tech_T2[10], double RKorr[6], std::string &IPOC)
{
	std::stringstream stream;
	stream << std::fixed << std::setprecision(4);

	// header
	stream << "<Sen Type=\"ImFree\">";// << "..";
	// body
	stream << "<EStr>Message from RSI TestServer</EStr>";// << "..";
	stream << "<Tech";
	for (int i = 1; i <= 10; i++)
	{
		stream << " T2" << i << "=\"" << Tech_T2[i - 1] << "\"";
	}
	stream << "/>";// << "..";
	stream << "<RKorr";
	const char cart[7] = "XYZABC";
	for (int i = 1; i <= 6; i++)
	{
		stream << " " << cart[i - 1] << "=\"" << RKorr[i - 1] << "\" ";
	}
	stream << "/>";// << "..";
	//stream << "<FREE>1000</FREE>";
	stream << "<DiO>125</DiO>";// << "..";
	// add ipoc, ack/timestamp
	stream << "<IPOC>" << IPOC << "</IPOC>";// << "..";

	stream << "</Sen>";
	
	IPOC = stream.str();
	std::cout << IPOC << std::endl;



}

bool extractTimestamp(char * sendBuf,std::string &IPOC)
{

	std::regex rgx("<IPOC>\\d+<\\/IPOC>");
	std::cmatch match;


	if (std::regex_search(sendBuf, match, rgx))
	{
		IPOC = match[0];
		return true;
	}
	else
		return false;

}

