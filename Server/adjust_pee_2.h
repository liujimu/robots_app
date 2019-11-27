/* 步态功能：调整落脚点
 * by liujimu, 2017-12-12
 */

/*将以下注释代码添加到xml文件*/
/*
            <ap default="ap_param">
                <ap_param type="group">
                    <totalCount abbreviation="t" type="int" default="3000"/>
                    <h abbreviation="h" type="double" default="0.03"/>
                </ap_param>
            </ap>
*/

#ifndef ADJUST_PEE_2_H
#define ADJUST_PEE_2_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct apParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
	double x{ 0.1 }; 
	double z{ 0 };
	double h{ 0.05 };
    double recoverPee[18]
    {  -0.30,   -0.50,   -0.52,
       -0.60,   -0.50,    0,
       -0.30,   -0.50,    0.52,
        0.30,   -0.50,   -0.52,
        0.60,   -0.50,    0,
        0.30,   -0.50,    0.52 };

};

auto adjustPeeParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto adjustPeeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // ADJUST_PEE_2_H
