﻿#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

#include "move_body.h"
#include "adjust_pee.h"
#include "swing.h"
#include "twist_waist.h"
#include "say_hello.h"
#include "find_joint_center.h"
#include "calibration.h"

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
	std::string xml_address;

	if (argc <= 1)
	{
		std::cout << "you did not type in robot name, in this case ROBOT-XIII will start" << std::endl;
		xml_address = "../../resource/Robot_XIII.xml";
	}
	else if (std::string(argv[1]) == "XIII")
	{
		xml_address = "../../resource/Robot_XIII.xml";
	}
	else
	{
		throw std::runtime_error("invalid robot name, please type in III or VIII");
	}
	
	auto &rs = aris::server::ControlServer::instance();
	

	rs.createModel<Robots::RobotTypeI>();
	rs.loadXml(xml_address.c_str());
	rs.addCmd("en", Robots::basicParse, nullptr);
	rs.addCmd("ds", Robots::basicParse, nullptr);
	rs.addCmd("hm", Robots::basicParse, nullptr);
	rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
	rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
	rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);
	
	//liujimu's gaits
	rs.addCmd("mb", moveBodyParse, moveBodyGait);
	rs.addCmd("ap", moveBodyParse, moveBodyGait);
	rs.addCmd("sw", swingParse, swingGait);
	rs.addCmd("tw", twistWaistParse, twistWaistGait);
	rs.addCmd("sh", sayHelloParse, sayHelloGait);
	rs.addCmd("fjc", findJointCenterParse, findJointCenterGait);
	rs.addCmd("cl", calibrationParse, calibrationGait);	

	rs.open();

	rs.setOnExit([&]() 
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.LoadFile(xml_address.c_str());
		auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
		if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
		rs.model().saveXml(*model_xml_ele);
		
		aris::core::stopMsgLoop();
	});
	aris::core::runMsgLoop();
	
	

	return 0;
}
