/* 步态功能：加载文本文件中的轨迹
* by liujimu, 2018-12-21
*/

/*将以下注释代码添加到xml文件*/
/*
            <lf default="lf_param">
                <lf_param type="group">
                    <totalCount abbreviation="t" type="int" default="720000"/>
                    <dir_param type="unique" default="y">
                        <y abbreviation="y"/>
                        <u abbreviation="u"/>
                        <w abbreviation="w"/>
                    </dir_param>
                </lf_param>
            </lf>
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

#include "akima_interpolation.h"

#include <aris_control_pipe.h>


#ifndef PI
#define PI 3.141592653589793
#endif

#ifndef DATA_SIZE
#define DATA_SIZE 1224
#endif // !DATA_SIZE


struct lfParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 720000 };
    int dir{ 1 };
};

//记录数据
struct LogParam
{
    double fce[6];
    double peb[6];
};

class LoadFile
{
public:
    static void loadFileParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
    static int loadFileGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

    static void recordData();

private:
    static double data_t[DATA_SIZE];
    static double data_y[DATA_SIZE];
    static double data_theta[DATA_SIZE];

    //记录数据
    static LogParam logParam;
    static aris::control::Pipe<LogParam> lfPipe;
    static std::thread lfThread;
};
