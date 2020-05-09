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
	bool read_char(unsigned char *recvChar);//��ȡ��������
	bool write_char(unsigned char *sendChar, int length);//�򴮿�д������
	
	void ReadPortThread(float Force[6]);

	HANDLE hComm;  /*����COM���*/
	DCB    m_dcb;  /*����״̬��Ϣ*/
	COMMTIMEOUTS m_CommTimeouts; /*��ʱ�����ṹ��*/
	BOOL  m_bCommReady;
	BOOL b_read;
	DWORD bytesToRead = 12;	//Ҫ��ȡ���ֽ���
	DWORD bytesReaded;		//�Ѷ�ȡ���ֽ���
	BOOL b_write;
	DWORD bytesToWrite = 4;  //Ҫд����ֽ���
	DWORD bytesWriten;		//��д���ֽ���
	const DWORD PACKAGELENGTH = 64;

	std::deque<unsigned char> recvData;
	float force[6];

private:
	bool open_port(UINT portNo);//�򿪴���
	bool setup_com();//���ô���,�Դ���ͨѶ���Ʋ�����������
	bool close_port();//�رմ���
	void get_useful_data(std::deque<unsigned char> *recvData, float Force[6]);
	

};



#endif // !_SERIALPORT_H
