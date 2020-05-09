#pragma once
#ifndef _SERIALPORT_H
#define _SERIALPORT_H

#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <string>
#include <deque>
#include <process.h>
#include <iterator>
#include <algorithm>
#include <time.h>


using namespace std;

class SPcom
{
public:
	SPcom();
	~SPcom();
	bool InitPort(UINT  portNo, DWORD  baud, BYTE  parity, BYTE  databits, BYTE  stopsbits);
	bool InitPort(UINT  portNo, const LPDCB& plDCB);
	bool read_char(unsigned char *recvChar);//读取串口数据
	bool write_char(unsigned char *sendChar, int length);//向串口写入数据
	
	void ReadPortThread(float Force[6]);

	HANDLE hComm;  /*串口COM句柄*/
	DCB    m_dcb;  /*串口状态信息*/
	COMMTIMEOUTS m_CommTimeouts; /*超时参数结构体*/
	BOOL  m_bCommReady;
	BOOL b_read;
	DWORD bytesToRead = 12;	//要读取的字节数
	DWORD bytesReaded;		//已读取的字节数
	BOOL b_write;
	DWORD bytesToWrite = 4;  //要写入的字节数
	DWORD bytesWriten;		//已写入字节数
	const DWORD PACKAGELENGTH = 64;

	std::deque<unsigned char> recvData;
	float force[6];

private:
	bool open_port(UINT portNo);//打开串口
	bool setup_com();//配置串口,对串口通讯控制参数进行设置
	bool close_port();//关闭串口
	void get_useful_data(std::deque<unsigned char> *recvData, float Force[6]);
	

};



#endif // !_SERIALPORT_H
