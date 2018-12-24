/* 步态功能：身体做正弦运动
* by liujimu, 2018-12-24
*/

/*将以下注释代码添加到xml文件*/
/*
            <sm default="sm_param">
                <sm_param type="group">
                    <totalCount abbreviation="t" type="int" default="120000"/>
                    <dir_param type="unique" default="y">
                        <y abbreviation="y"/>
                        <u abbreviation="u"/>
                        <w abbreviation="w"/>
                    </dir_param>
                    <cycle abbreviation="c" type="double" default="6"/>
                    <radius abbreviation="r" type="double" default="0.6042"/>
                </sm_param>
            </sm>
*/

#pragma once
#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct smParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 120000 };
    int dir{ 1 };
    double cycle{ 6 };
    double amp{ 0.05 };
    double radius{ 0.6042 };
};

auto sineMotionParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto sineMotionGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
