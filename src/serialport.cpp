#include "serialport.h"

using namespace std;


/* 当串口无数据时,sleep至下次查询间隔的时间,单位:秒 */
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
		
	/** 是否有错误发生 */	
	BOOL bIsSuccess = TRUE; 	
	/* 
	在此可以设置输入输出的缓冲区大小,如果不设置,则系统会设置默认值.	  
	自己设置缓冲区大小时,要注意设置稍大一些,避免缓冲区溢出	
	*/	
	///*
	if (bIsSuccess )	
	{	
		bIsSuccess = SetupComm(hComm,64,64);	
	}
	//*/
	//DBC参数设置
	if (bIsSuccess)
	{
		/* 使用DCB参数配置串口状态 */
		GetCommState(hComm, &m_dcb);
		m_dcb.BaudRate = baud;/*波特率:230400*/
		m_dcb.ByteSize = databits;/*通信数据字节位数:8*/
		m_dcb.Parity = parity;    /*指定奇偶校验位:0*/
		m_dcb.StopBits = stopsbits;/*指定停止位的位数:0*/
		/* 开启RTS flow控制 */
		m_dcb.fRtsControl = RTS_CONTROL_ENABLE;
		bIsSuccess = SetCommState(hComm, &m_dcb); /*设置串口*/
	}
	
	/* 设置串口的超时时间,均设为0,表示不使用超时限制 */	
	if (bIsSuccess) 
	{ 
		if (GetCommTimeouts(hComm, &m_CommTimeouts) == 0) /*判断超时参数是否获取*/
		{
			CloseHandle(hComm);
		}
		m_CommTimeouts.ReadIntervalTimeout = 0;
		m_CommTimeouts.ReadTotalTimeoutConstant = 0;
		m_CommTimeouts.ReadTotalTimeoutMultiplier = 0;
		m_CommTimeouts.WriteTotalTimeoutConstant = 10;
		m_CommTimeouts.WriteTotalTimeoutMultiplier = 0;
		SetCommTimeouts(hComm, &m_CommTimeouts);/*设置超时*/
		bIsSuccess = SetCommTimeouts(hComm, &m_CommTimeouts); 
	} 		
	/*  清空串口缓冲区 */	
	PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT); 	
		
	return bIsSuccess == TRUE;
}

bool SPcom::InitPort(UINT portNo, const LPDCB& plDCB) 
{	
	/* 打开指定串口 */	
	if (!open_port(portNo)) 
	{ 
		return false; 
	} 	
	 	
	/* 配置串口参数 */
	if (!SetCommState(hComm, plDCB)) 
	{ 
		return false;
	} 	
	/*  清空串口缓冲区 */	
	PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT); 	
	 	
	return true; 
}


/*打开串口，参数为串口名称的字符串。如：“COM1".*/
bool SPcom::open_port(UINT portNo)
{
	
	/* 把串口的编号转换为设备名 */
	char szPort[50];
	sprintf_s(szPort, "COM%d", portNo);

	hComm = CreateFileA(szPort,				/* 合法的标准设备名,//./COM1,//./COM2等 */
						GENERIC_READ | GENERIC_WRITE,   /* 访问模式,可同时读写 */
						0,                              /* 共享模式,0表示不共享，由于串口不能共享，必须设为零 */
						NULL,                           /*安全性设置,一般使用NULL */
						OPEN_EXISTING,                  /* 创建标志，对串口操作该参数必须置为OPEN_EXISTING,否则创建失败 */
						0,								/*属性描述，用于指定该串口是否进行异步操作，该值为FILE_FLAG_OVERLAPPED，表示使用异步的I/O；该值为0，表示同步I/O操作；*/
						NULL);
	/* 如果打开失败，释放资源并返回 */
	if (hComm == INVALID_HANDLE_VALUE) 
	{
		std::cout << "打开串口失败！" << std::endl;
		return false;
	}
	else
		std::cout << "打开串口成功！" << std::endl;
		return true;
	
}

bool SPcom::setup_com()
{
	SetupComm(hComm, 48, 48);  //输入缓冲区和输出缓冲区的大小都是1024

	//DBC参数设置
	if (GetCommState(hComm, &m_dcb) == 0) /*判断串口信息是否获取*/
	{
		CloseHandle(hComm);
	}
	m_dcb.BaudRate = 230400;/*波特率*/
	m_dcb.ByteSize = 8;/*通信数据字节位数*/
	m_dcb.Parity = NOPARITY;    /*指定奇偶校验位*/
	m_dcb.StopBits = ONESTOPBIT;/*指定停止位的位数*/
	SetCommState(hComm, &m_dcb); /*设置串口*/

	//超时参数设置
	if (GetCommTimeouts(hComm, &m_CommTimeouts) == 0) /*判断超时参数是否获取*/
	{
		CloseHandle(hComm);
	}
	m_CommTimeouts.ReadIntervalTimeout = 0;
	m_CommTimeouts.ReadTotalTimeoutConstant = 5000;
	m_CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	m_CommTimeouts.WriteTotalTimeoutConstant = 10;
	m_CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	SetCommTimeouts(hComm, &m_CommTimeouts);/*设置超时*/

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
		/* 获取错误码,可以根据该错误码查出错误原因 */		
		DWORD dwError = GetLastError(); 		
		/* 清空串口缓冲区 */		
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
	/* 向缓冲区写入指定量的数据 */
	b_write = WriteFile(hComm, sendChar, bytesToWrite, &bytesWriten, 0);
	if (!b_write)
	{
		DWORD dwError = GetLastError();		
		/* 清空串口缓冲区 */		
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
		
	CloseHandle(hComm);/*关闭串口*/
	return true;
}

//读取串口数据并解析
void SPcom::ReadPortThread(float Force[6])
{
	BOOL b_read;
	DWORD bytesReaded;
	DWORD bytesToRead = 12 ;
	const DWORD PACKAGELENGTH = 64;
	UINT cycleNum = 1;

	//while (hComm != INVALID_HANDLE_VALUE)
	//{
	DWORD dwError = 0;  /* 错误码 */
	COMSTAT  comstat;   /* COMSTAT结构体,记录通信设备的状态信息 */
	memset(&comstat, 0, sizeof(COMSTAT));
	DWORD BytesInQue;
		
	PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	///*
	//写入一个发送一个
	unsigned char sendData[4];
	sendData[0] = (BYTE)0x49;
	sendData[1] = (BYTE)0xAA;
	sendData[2] = (BYTE)0x0D;
	sendData[3] = (BYTE)0x0A;
	write_char(sendData, sizeof(sendData));
	//*/
		
	//在调用ReadFile和WriteFile之前,通过本函数清除以前遗留的错误标志 
	if (ClearCommError(hComm, &dwError, &comstat))
	{
		BytesInQue = comstat.cbInQue; // 获取在输入缓冲区中的字节数 
	}
	//如果串口输入缓冲区中无数据,则休息一会再查询 ，如果1个循环内的读取及解析数据完成后，串口缓冲区还没收到数据则间隔SLEEP_TIME_INTERVAL=1ms读取
	if (BytesInQue == 0)
	{
		Sleep(SLEEP_TIME_INTERVAL);
		//continue;   //数据读取的周期增大
	}
	else if (BytesInQue >= PACKAGELENGTH)
	{
		PurgeComm(hComm, PURGE_RXCLEAR|PURGE_TXCLEAR);  //清除串口缓冲区 PURGE_RXCLEAR: 清除接收缓冲区的所有数据。
		//break;
	}
		
		
		
	//从串口读取数据
	unsigned char buf[PACKAGELENGTH];
	b_read = ReadFile(hComm, buf, bytesToRead, &bytesReaded, NULL);  
	if (!b_read)
	{
		printf("无法从串口读取数据！");
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
		//从缓冲区双向队列中提取数据
		get_useful_data(&recvData, Force);
		//printf("Fx= %2f N,Fy= %2f N,Fz= %2f N,Mx= %2f N.M,My= %2f N.M,Mz= %2f N.M\n", Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
		
	}
		
		
	//}
	
}



//从数据缓存块中提取可靠的数据
void SPcom::get_useful_data(std::deque<unsigned char> *recvData, float Force[6])
{
	unsigned char data[27];
	unsigned int DataTemp;
	unsigned char t[4];
	

	int ReceivedDataLangth = recvData->size();  //缓存目前收到数据的长度，以免循环过程中有新数据写入或读出影响操作
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
