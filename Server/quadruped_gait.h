/* 步态功能：抬起两只前腿，用后四条腿走路
* by liujimu, 2018-8-4
*/

/*将以下注释代码添加到xml文件*/
/*
            <qg default="qg_param">
                <qg_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
                    <bodyUp abbreviation="u" type="double" default="0.03"/>
                    <bodyBack abbreviation="b" type="double" default="0.14"/>
                    <bodyPitch abbreviation="p" type="double" default="5"/>
                    <frontFootInnerOffset abbreviation="f" type="double" default="0.2"/>
                    <middleFootForwardOffset abbreviation="m" type="double" default="0.18"/>
                    <rearFootOuterOffset abbreviation="r" type="double" default="0.1"/>
                    <walkStepLength abbreviation="d" type="double" default="0.08"/>
                    <walkStepHeight abbreviation="h" type="double" default="0.03"/>
                    <n abbreviation="n" type="int" default="1"/>
                </qg_param>
            </qg>
*/

#ifndef QUADRUPED_GAIT_H
#define QUADRUPED_GAIT_H

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

struct qgParam final :public aris::server::GaitParamBase
{
    std::int32_t total_count{ 1000 };
    double body_up{ 0.03 };
    double body_back{ 0.14 };
    double body_pitch{ PI / 36 };
    double front_foot_upward_offset{ 0.02 };
    double front_foot_inner_offset{ 0.2 };
    double front_foot_upward_offset_2{ 0.08 };
    double front_foot_backward_offset_2{ 0.15 };
    double middle_foot_forward_offset{ 0.18 };
    double middle_foot_inner_offset{ 0.25 };
    double rear_foot_outer_offset{ 0.1 };
    double body_down{ 0.03 };
    double body_sidesway{ 0.1 };
    double walk_step_length{ 0.08 };
    double walk_step_height{ 0.03 };
    std::int32_t n{ 1 };
};

auto quadrupedGaitParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto quadrupedGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // QUADRUPED_GAIT_H
