/* 步态功能：身体俯仰
* by liujimu, 2016-12-12
*/

/*将以下注释代码添加到xml文件*/
/*
            <sw default="sw_param">
                <sw_param type="group">
                    <totalCount abbreviation="t" type="int" default="8000"/>
                    <xAngle abbreviation="x" type="double" default="0"/>
                    <zAngle abbreviation="z" type="double" default="0"/>
                    <yAngle abbreviation="y" type="double" default="0"/>
                    <rDistance abbreviation="r" type="double" default="0.6"/>
                    <yDistance abbreviation="d" type="double" default="0.05"/>
                </sw_param>
            </sw>
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

class PitchState
{
public:
    static PitchState& getState()
    {
        static PitchState s;
        return s;
    }
    bool& isStopping() { return isStopping_; }
private:
    bool isStopping_{ true };
    PitchState() = default;
};

struct pitchParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
    double y{ 0 };
    double z{ 0 };
	double a{ 25 * PI / 180 };
    double h{ 0.05 };
    double prePee[18]
    {  -0.5, -0.9, -0.6,
       -0.8, -0.9,  0,
       -0.9, -0.9,  0.6,
        0.5, -0.9, -0.6,
        0.8, -0.9,  0,
        0.9, -0.9,  0.6};
};

auto pitchParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto pitchGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
