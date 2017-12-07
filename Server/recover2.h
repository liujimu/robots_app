/* 步态功能：机器人悬空状态下抬高所有腿
 * by liujimu, 2017-12-7
 */

/*将以下注释代码添加到xml文件*/
/*
            <rc2 default="rc_param">
                <rc2_param type="group">
                    <leg_param type="unique" default="all">
                        <all abbreviation="a"/>
                        <first abbreviation="f"/>
                        <second abbreviation="s"/>
                        <leg abbreviation="l" type="intArray" default="0"/>
                    </leg_param>
                    <stepNumber abbreviation="n" type="intArray" default="1"/>
                    <totalCount abbreviation="t" type="intArray" default="1500"/>
                    <margin_offset abbreviation="m" type="double" default="0.005"/>
                </rc2_param>
            </rc2>
*/

#ifndef RECOVER2_H
#define RECOVER2_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <stdlib.h>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct rc2Param final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 3000 };
    bool active_leg[6]{ true,true,true,true,true,true };
    double margin_offset{ 0.005 };//meter
    double targetPee[2][18]
    {
        {   -0.43,   -0.12,   -0.7448,
            -0.86,   -0.12,    0,
            -0.43,   -0.12,    0.7448,
            0.43,   -0.12,   -0.7448,
            0.86,   -0.12,    0,
            0.43,   -0.12,    0.7448 },
       {    -0.30,   -0.50,   -0.52,
            -0.60,   -0.50,    0,
            -0.30,   -0.50,    0.52,
            0.30,   -0.50,   -0.52,
            0.60,   -0.50,    0,
            0.30,   -0.50,    0.52 };

    };
    int stepNumber{ 1 };
};

auto recover2Parse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto recover2Gait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // RECOVER2_H
