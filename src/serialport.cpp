#include "serialport.h"

using namespace std;


/* ������������ʱ,sleep���´β�ѯ�����ʱ��,��λ:�� */
const UINT SLEEP_TIME_INTERVAL = 1;

SPcom::SPcom()
{
	hComm = INVALID_HANDLE_VALUE;

}

SPcom::~SPcom()
{

}

bool SPcom::InitPort(UINT portNo /*= 1*/, DWORD baud /*= 230400*/, BYTE parity /* = 0*/, BYTE databits /*= 8*/, BYTE stopsbits /*= 0*/) 
{ 		

	if (!open_port(portNo)) 
	{ 
		return false; 
	} 	
		
	/** �Ƿ��д����� */	
	BOOL bIsSuccess = TRUE; 	
	/* 
	�ڴ˿���������������Ļ�������С,���������,��ϵͳ������Ĭ��ֵ.	  
	�Լ����û�������Сʱ,Ҫע�������Դ�һЩ,���⻺�������	
	*/	
	///*
	if (bIsSuccess )	
	{	
		bIsSuccess = SetupComm(hComm,64,64);	
	}
	//*/
	//DBC��������
	if (bIsSuccess)
	{
		/* ʹ��DCB�������ô���״̬ */
		GetCommState(hComm, &m_dcb);
		m_dcb.BaudRate = baud;/*������:230400*/
		m_dcb.ByteSize = databits;/*ͨ�������ֽ�λ��:8*/
		m_dcb.Parity = parity;    /*ָ����żУ��λ:0*/
		m_dcb.StopBits = stopsbits;/*ָ��ֹͣλ��λ��:0*/
		/* ����RTS flow���� */
		m_dcb.fRtsControl = RTS_CONTROL_ENABLE;
		bIsSuccess = SetCommState(hComm, &m_dcb); /*���ô���*/
	}
	
	/* ���ô��ڵĳ�ʱʱ��,����Ϊ0,��ʾ��ʹ�ó�ʱ���� */	
	if (bIsSuccess) 
	{ 
		if (GetCommTimeouts(hComm, &m_CommTimeouts) == 0) /*�жϳ�ʱ�����Ƿ��ȡ*/
		{
			CloseHandle(hComm);
		}
		m_CommTimeouts.ReadIntervalTimeout = 0;
		m_CommTimeouts.ReadTotalTimeoutConstant = 0;
		m_CommTimeouts.ReadTotalTimeoutMultiplier = 0;
		m_CommTimeouts.WriteTotalTimeoutConstant = 10;
		m_CommTimeouts.WriteTotalTimeoutMultiplier = 0;
		SetCommTimeouts(hComm, &m_CommTimeouts);/*���ó�ʱ*/
		bIsSuccess = SetCommTimeouts(hComm, &m_CommTimeouts); 
	} 		
	/*  ��մ��ڻ����� */	
	PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT); 	
		
	return bIsSuccess == TRUE;
}

bool SPcom::InitPort(UINT portNo, const LPDCB& plDCB) 
{	
	/* ��ָ������ */	
	if (!open_port(portNo)) 
	{ 
		return false; 
	} 	
	 	
	/* ���ô��ڲ��� */
	if (!SetCommState(hComm, plDCB)) 
	{ 
		return false;
	} 	
	/*  ��մ��ڻ����� */	
	PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT); 	
	 	
	return true; 
}


/*�򿪴��ڣ�����Ϊ�������Ƶ��ַ������磺��COM1".*/
bool SPcom::open_port(UINT portNo)
{
	
	/* �Ѵ��ڵı��ת��Ϊ�豸�� */
	char szPort[50];
	sprintf_s(szPort, "COM%d", portNo);

	hComm = CreateFileA(szPort,				/* �Ϸ��ı�׼�豸��,//./COM1,//./COM2�� */
						GENERIC_READ | GENERIC_WRITE,   /* ����ģʽ,��ͬʱ��д */
						0,                              /* ����ģʽ,0��ʾ���������ڴ��ڲ��ܹ���������Ϊ�� */
						NULL,                           /*��ȫ������,һ��ʹ��NULL */
						OPEN_EXISTING,                  /* ������־���Դ��ڲ����ò���������ΪOPEN_EXISTING,���򴴽�ʧ�� */
						0,								/*��������������ָ���ô����Ƿ�����첽��������ֵΪFILE_FLAG_OVERLAPPED����ʾʹ���첽��I/O����ֵΪ0����ʾͬ��I/O������*/
						NULL);
	/* �����ʧ�ܣ��ͷ���Դ������ */
	if (hComm == INVALID_HANDLE_VALUE) 
	{
		std::cout << "�򿪴���ʧ�ܣ�" << std::endl;
		return false;
	}
	else
		std::cout << "�򿪴��ڳɹ���" << std::endl;
		return true;
	
}

bool SPcom::setup_com()
{
	SetupComm(hComm, 48, 48);  //���뻺����������������Ĵ�С����1024

	//DBC��������
	if (GetCommState(hComm, &m_dcb) == 0) /*�жϴ�����Ϣ�Ƿ��ȡ*/
	{
		CloseHandle(hComm);
	}
	m_dcb.BaudRate = 230400;/*������*/
	m_dcb.ByteSize = 8;/*ͨ�������ֽ�λ��*/
	m_dcb.Parity = NOPARITY;    /*ָ����żУ��λ*/
	m_dcb.StopBits = ONESTOPBIT;/*ָ��ֹͣλ��λ��*/
	SetCommState(hComm, &m_dcb); /*���ô���*/

	//��ʱ��������
	if (GetCommTimeouts(hComm, &m_CommTimeouts) == 0) /*�жϳ�ʱ�����Ƿ��ȡ*/
	{
		CloseHandle(hComm);
	}
	m_CommTimeouts.ReadIntervalTimeout = 0;
	m_CommTimeouts.ReadTotalTimeoutConstant = 5000;
	m_CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	m_CommTimeouts.WriteTotalTimeoutConstant = 10;
	m_CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	SetCommTimeouts(hComm, &m_CommTimeouts);/*���ó�ʱ*/

	if(GetCommState(hComm, &m_dcb) != 0 && GetCommTimeouts(hComm, &m_CommTimeouts) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool SPcom::read_char(unsigned char *recvChar)
{
	b_read = TRUE;
	bytesReaded = 0;
	if (hComm == INVALID_HANDLE_VALUE)
	{
		return false;

	}
	b_read = ReadFile(hComm, recvChar, bytesToRead, &bytesReaded, NULL);
	if (!b_read)
	{
		/* ��ȡ������,���Ը��ݸô�����������ԭ�� */		
		DWORD dwError = GetLastError(); 		
		/* ��մ��ڻ����� */		
		PurgeComm(hComm, PURGE_RXCLEAR | PURGE_RXABORT);		
		return false;
	}
	return (bytesReaded == 12);
}

bool SPcom::write_char(unsigned char *sendChar, int length)
{
	b_write = TRUE;	
	bytesWriten = 0;
	bytesToWrite = length;
	if (hComm == INVALID_HANDLE_VALUE) 
	{
		return false; 
	} 	 	
	/* �򻺳���д��ָ���������� */
	b_write = WriteFile(hComm, sendChar, bytesToWrite, &bytesWriten, 0);
	if (!b_write)
	{
		DWORD dwError = GetLastError();		
		/* ��մ��ڻ����� */		
		PurgeComm(hComm, PURGE_RXCLEAR | PURGE_RXABORT);				
		return false;
		
	}
	
	return true;
}

bool SPcom::close_port()
{
	if (hComm == INVALID_HANDLE_VALUE)
	{
		return true;
	}
		
	CloseHandle(hComm);/*�رմ���*/
	return true;
}

//��ȡ�������ݲ�����
void SPcom::ReadPortThread(float Force[6])
{
	BOOL b_read;
	DWORD bytesReaded;
	DWORD bytesToRead = 12 ;
	const DWORD PACKAGELENGTH = 64;
	UINT cycleNum = 1;

	//while (hComm != INVALID_HANDLE_VALUE)
	//{
	DWORD dwError = 0;  /* ������ */
	COMSTAT  comstat;   /* COMSTAT�ṹ��,��¼ͨ���豸��״̬��Ϣ */
	memset(&comstat, 0, sizeof(COMSTAT));
	DWORD BytesInQue;
		
	PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	///*
	//д��һ������һ��
	unsigned char sendData[4];
	sendData[0] = (BYTE)0x49;
	sendData[1] = (BYTE)0xAA;
	sendData[2] = (BYTE)0x0D;
	sendData[3] = (BYTE)0x0A;
	write_char(sendData, sizeof(sendData));
	//*/
		
	//�ڵ���ReadFile��WriteFile֮ǰ,ͨ�������������ǰ�����Ĵ����־ 
	if (ClearCommError(hComm, &dwError, &comstat))
	{
		BytesInQue = comstat.cbInQue; // ��ȡ�����뻺�����е��ֽ��� 
	}
	//����������뻺������������,����Ϣһ���ٲ�ѯ �����1��ѭ���ڵĶ�ȡ������������ɺ󣬴��ڻ�������û�յ���������SLEEP_TIME_INTERVAL=1ms��ȡ
	if (BytesInQue == 0)
	{
		Sleep(SLEEP_TIME_INTERVAL);
		//continue;   //���ݶ�ȡ����������
	}
	else if (BytesInQue >= PACKAGELENGTH)
	{
		PurgeComm(hComm, PURGE_RXCLEAR|PURGE_TXCLEAR);  //������ڻ����� PURGE_RXCLEAR: ������ջ��������������ݡ�
		//break;
	}
		
		
		
	//�Ӵ��ڶ�ȡ����
	unsigned char buf[PACKAGELENGTH];
	b_read = ReadFile(hComm, buf, bytesToRead, &bytesReaded, NULL);  
	if (!b_read)
	{
		printf("�޷��Ӵ��ڶ�ȡ���ݣ�");
	}
	else
	{
		for (int i = 0; i < bytesToRead; i++)
		{
			//printf("%02X ",buf[i]) ;
			recvData.push_back(buf[i]);
			
		}
		///*
		//std::cout << BytesInQue << ";" << std::endl;
		//�ӻ�����˫���������ȡ����
		get_useful_data(&recvData, Force);
		//printf("Fx= %2f N,Fy= %2f N,Fz= %2f N,Mx= %2f N.M,My= %2f N.M,Mz= %2f N.M\n", Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
		
	}
		
		
	//}
	
}



//�����ݻ��������ȡ�ɿ�������
void SPcom::get_useful_data(std::deque<unsigned char> *recvData, float Force[6])
{
	unsigned char data[27];
	unsigned int DataTemp;
	unsigned char t[4];
	

	int ReceivedDataLangth = recvData->size();  //����Ŀǰ�յ����ݵĳ��ȣ�����ѭ����������������д������Ӱ�����
	if ((ReceivedDataLangth >= 12) && (recvData->at(10) == 0x0d) && (recvData->at(11) == 0x0a))
	{
		for (int i = 0; i < 12; i++)
		{
			data[i] = recvData->front();
			recvData->pop_front();
		}
		DataTemp = 0;
		if ((data[1] & 0x80) > 0)
		{
			DataTemp = data[1];
			DataTemp = DataTemp | 0xFFFFFF00;
			DataTemp = DataTemp << 4;
			DataTemp = DataTemp | (UINT)(data[2] >> 4);
		}
		else
		{
			DataTemp = data[1];
			DataTemp = DataTemp << 4;
			DataTemp = DataTemp | (UINT)(data[2] >> 4);
		}
		Force[0] = (int)DataTemp * (float)0.009765625;
		DataTemp = 0;
		if ((data[2] & 0x08) > 0)
		{
			DataTemp = data[2];
			DataTemp = DataTemp | 0xFFFFFFF0;
			DataTemp = DataTemp << 8;
			DataTemp = DataTemp | data[3];
		}
		else
		{
			DataTemp = data[2];
			DataTemp = DataTemp & 0x0F;
			DataTemp = DataTemp << 8;
			DataTemp = DataTemp | data[3];
		}
		Force[1] = (int)DataTemp * (float)0.009765625;
		DataTemp = 0;
		if ((data[4] & 0x80) > 0)
		{
			DataTemp = data[4];
			DataTemp = DataTemp | 0xFFFFFF00;
			DataTemp = DataTemp << 4;
			DataTemp = DataTemp | (UINT)(data[5] >> 4);
		}
		else
		{
			DataTemp = data[4];
			DataTemp = DataTemp << 4;
			DataTemp = DataTemp | (UINT)(data[5] >> 4);
		}
		Force[2] = (int)DataTemp * (float)0.009765625;
		DataTemp = 0;
		if ((data[5] & 0x08) > 0)
		{
			DataTemp = data[5];
			DataTemp = DataTemp | 0xFFFFFFF0;
			DataTemp = DataTemp << 8;
			DataTemp = DataTemp | data[6];
		}
		else
		{
			DataTemp = data[5];
			DataTemp = DataTemp & 0x0F;
			DataTemp = DataTemp << 8;
			DataTemp = DataTemp | data[6];
		}
		Force[3] = (int)DataTemp * (float)0.000390625;
		DataTemp = 0;
		if ((data[7] & 0x80) > 0)
		{
			DataTemp = data[7];
			DataTemp = DataTemp | 0xFFFFFF00;
			DataTemp = DataTemp << 4;
			DataTemp = DataTemp | (UINT)(data[8] >> 4);
		}
		else
		{
			DataTemp = data[7];
			DataTemp = DataTemp << 4;
			DataTemp = DataTemp | (UINT)(data[8] >> 4);
		}
		Force[4] = (int)DataTemp * (float)0.000390625;
		DataTemp = 0;
		if ((data[8] & 0x08) > 0)
		{
			DataTemp = data[8];
			DataTemp = DataTemp | 0xFFFFFFF0;
			DataTemp = DataTemp << 8;
			DataTemp = DataTemp | data[9];
		}
		else
		{
			DataTemp = data[8];
			DataTemp = DataTemp & 0x0F;
			DataTemp = DataTemp << 8;
			DataTemp = DataTemp | data[9];
		}
		Force[5] = (int)DataTemp * (float)0.000390625;
		//printf("Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n", Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
		
		for (int i = 0; i < 6; i++)
		{
			Force[i] *= 10;
		}
		//printf("Fx= %2f N,Fy= %2f N,Fz= %2f N,Mx= %2f N.M,My= %2f N.M,Mz= %2f N.M\n", Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
	}
	else if ((ReceivedDataLangth >= 12) && (ReceivedDataLangth < 50))
	{
		if (recvData->at(0) == 0x0a)
			recvData->pop_front();
		else
		{
			int i = 0;
			while ((i <= 12) && (recvData->at(0) != 0x0d) && (recvData->at(1) != 0x0a))
			{
				recvData->pop_front();
				i++;
			}
			if (recvData->size() >= 2)
			{
				recvData->pop_front();
				recvData->pop_front();
			}

		}
	}
	else if (ReceivedDataLangth >= 50)
		recvData->clear();
	else if (ReceivedDataLangth < 12)
	recvData->clear();
}
