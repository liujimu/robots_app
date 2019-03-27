/* 步态功能：跨越障碍
 * by liujimu, 2016-12-12
 */

/*将以下注释代码添加到xml文件*/
/*
            <co default="co_param">
                <co_param type="group">
                    <totalCount abbreviation="t" type="int" default="3000"/>
                    <n abbreviation="n" type="int" default="4"/>
                    <d abbreviation="d" type="double" default="-0.7"/>
                    <h abbreviation="h" type="double" default="0.25"/>
                    <y abbreviation="y" type="double" default="0.12"/>
                </co_param>
            </co>
*/

#ifndef CROSS_OBSTACLE_H
#define CROSS_OBSTACLE_H

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

struct coParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
	std::int32_t n{ 4 };
	double d{ -0.7 }; // d>0向右跨，d<0向左跨
	double h{ 0.25 };
	double y{ 0.12 };
	// 0.20m台阶：n=4; d=0.7; h=0.25; y=0.12;
	// 0.25m台阶：n=5; d=0.5; h=0.28; y=0.15;
};

auto crossObstacleParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto crossObstacleGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // CROSS_OBSTACLE_H
