/* 步态功能：移动单条腿
 * by liujimu, 2017-4-11
 */

/*将以下注释代码添加到xml文件*/
/*
            <mol default="mol_param">
                <mol_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
                    <d abbreviation="d" type="double" default="0"/>
                    <h abbreviation="h" type="double" default="0.05"/>
                    <legID abbreviation="l" type="int" default="0"/>
                </mol_param>
            </mol>
*/

#ifndef MOVE_ONE_LEG_H
#define MOVE_ONE_LEG_H

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

struct molParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
    double d{ 0.3 };
    double h{ 0.05 };
    int legID{ 0 };
};

auto moveOneLegParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto moveOneLegGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // MOVE_ONE_LEG_H
