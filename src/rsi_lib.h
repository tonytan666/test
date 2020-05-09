#pragma once
#ifndef _RSI_LIB_
#define _RSI_LIB_

#include <iostream>
#include "tinyxml2.h"
#include <vector>
using namespace std;
using namespace tinyxml2;

class RSIState
{
public:
	char * xml_str;// = "<Rob Type = \"KUKA\"> <RIst X = \"2131.7874\" Y = \"150.7920\" Z = \"1112.0045\" A = \"179.2204\" B = \"0.6697\" C = \"179.5603\"/> <RSol X = \"2131.7910\" Y = \"150.7917\" Z = \"1111.9225\" A = \"179.2200\" B = \"0.6700\" C = \"179.5600\"/><Delay D = \"92\"/> <Tech C11 = \"0.0000\" C12 = \"0.0000\" C13 = \"0.0000\" C14 = \"0.0000\" C15 = \"0.0000\" C16 = \"0.0000\"  C17 = \"0.0000\" C18 = \"0.0000\" C19 = \"0.0000\" C110 = \"0.0000\"/> <DiL> 0 </DiL> <Digout o1 = \"0\" o2 = \"0\" o3 = \"0\"/><Source1>-29.3893</Source1> <IPOC>5839994 </IPOC> </Rob>";
public:
	std::vector<char>axis_pos = { 0,0,0,0,0,0 };
	std::vector<char>init_axis_pos = { 0,0,0,0,0,0 };
	std::vector<char>cart_pos = { 0,0,0,0,0,0 };
	std::vector<char>init_cart_pos = { 0,0,0,0,0,0 };
	unsigned long long ipoc;
	double act_axis_pos[6] = { 0,0,0,0,0,0 };
	double the_axis_pos[6] = { 0,0,0,0,0,0 }; 
	double act_cart_pos[6] = { 0,0,0,0,0,0 };
	double the_cart_pos[6] = { 0,0,0,0,0,0 };
	bool rsi_is_connected = false;
	RSIState()
	{

	}
	~RSIState()
	{

	}
	void get_tag_data(char * xml_str)
	{
		
		// Parse message from Robot

		 tinyxml2::XMLDocument recvBufDoc;

		int r = recvBufDoc.Parse(xml_str);
		if (r != XML_SUCCESS)
		{
			std::cout << "解析XML文件失败！" << std::endl;
		}

		// Get the Rob node:

		tinyxml2::XMLElement * Rob = recvBufDoc.RootElement(); //FirstChildElement("Rob")

		/*
		// Extract axis specific actual position

		tinyxml2::XMLElement* AIPos = Rob->FirstChildElement("AIPos");
		AIPos->Attribute("A1", &axis_pos[0]);
		AIPos->Attribute("A2", &axis_pos[1]);
		AIPos->Attribute("A3", &axis_pos[2]);
		AIPos->Attribute("A4", &axis_pos[3]);
		AIPos->Attribute("A5", &axis_pos[4]);
		AIPos->Attribute("A6", &axis_pos[5]);
		for (int i = 0; i < 6; i++)
		{
			act_axis_pos[i] = std::atof(&axis_pos[i]);
		}
		// Extract axis specific setpoint position

		tinyxml2::XMLElement* ASPos = Rob->FirstChildElement("ASPos");
		ASPos->Attribute("A1", &init_axis_pos[0]);
		ASPos->Attribute("A2", &init_axis_pos[1]);
		ASPos->Attribute("A3", &init_axis_pos[2]);
		ASPos->Attribute("A4", &init_axis_pos[3]);
		ASPos->Attribute("A5", &init_axis_pos[4]);
		ASPos->Attribute("A6", &init_axis_pos[5]);
		for (int i = 0; i < 6; i++)
		{
			the_axis_pos[i] = std::atof(&init_axis_pos[i]);
		}
		*/

		// Extract cartesian actual position

		tinyxml2::XMLElement* RIst = Rob->FirstChildElement("RIst");
		/*const tinyxml2::XMLAttribute *xAttr;
		xAttr = RIst->FindAttribute("X");
		act_cart_pos[0] = xAttr->DoubleValue();*/
		
		act_cart_pos[0] = RIst->DoubleAttribute("X");
		act_cart_pos[1] = RIst->DoubleAttribute("Y");
		act_cart_pos[2] = RIst->DoubleAttribute("Z");
		act_cart_pos[3] = RIst->DoubleAttribute("A");
		act_cart_pos[4] = RIst->DoubleAttribute("B");
		act_cart_pos[5] = RIst->DoubleAttribute("C");
		//std::cout << act_cart_pos[0] << "," << act_cart_pos[1] << "," << act_cart_pos[2] << "," << act_cart_pos[3] << "," << act_cart_pos[4] << "," << act_cart_pos[5] << std::endl;
		
		// Extract cartesian setpoint position

		tinyxml2::XMLElement* RSol = Rob->FirstChildElement("RSol");
		the_cart_pos[0] = RSol->DoubleAttribute("X");
		the_cart_pos[1] = RSol->DoubleAttribute("Y");
		the_cart_pos[2] = RSol->DoubleAttribute("Z");
		the_cart_pos[3] = RSol->DoubleAttribute("A");
		the_cart_pos[4] = RSol->DoubleAttribute("B");
		the_cart_pos[5] = RSol->DoubleAttribute("C");
		
		// Get the IPOC timestamp

		tinyxml2::XMLElement* IPOC = Rob->FirstChildElement("IPOC");

		ipoc = std::atoll(IPOC->FirstChild()->Value());
		//std::cout << ipoc << std::endl;
		
		//获取RSI的连接状态
		tinyxml2::XMLElement * Digout = Rob->FirstChildElement("Digout");
		rsi_is_connected = Digout->DoubleAttribute("o1");
		//std::cout << rsi_is_connected << std::endl;
	}

};
#endif // !_RSI_LIB_
