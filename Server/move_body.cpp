#include "move_body.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto moveBodyParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    mbParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "x")
        {
            param.x = std::stod(i.second);
        }
        else if (i.first == "y")
        {
            param.y = std::stod(i.second);
        }
        else if (i.first == "z")
        {
            param.z = std::stod(i.second);
        }
        else if (i.first == "pitch")
        {
            param.pitch = std::stod(i.second) / 180 * PI;
        }
        else if (i.first == "yaw")
        {
            param.yaw = std::stod(i.second) / 180 * PI;
        }
        else if (i.first == "roll")
        {
            param.roll = std::stod(i.second) / 180 * PI;
        }
    }

    msg.copyStruct(param);
}

auto moveBodyGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const mbParam &>(param_in);

    //行程检测
#ifdef UNIX
    static aris::server::ControlServer &cs = aris::server::ControlServer::instance();
#endif

    const double safetyOffset{ 0 };
    double pinUpBound[18]{ 0 };
    double pinLowBound[18]{ 0 };

#ifdef WIN32
    //老虎电机教育机器人的行程参数
    for (int i = 0; i < 18; i += 3)
    {
        pinUpBound[i] = 0.7745 - safetyOffset;
        pinUpBound[i + 1] = 0.7900 - safetyOffset;
        pinUpBound[i + 2] = 0.7900 - safetyOffset;
        pinLowBound[i] = 0.5675 + safetyOffset;
        pinLowBound[i + 1] = 0.5830 + safetyOffset;
        pinLowBound[i + 2] = 0.5830 + safetyOffset;
    }
#endif
#ifdef UNIX
    for (int i = 0; i < 18; ++i)
    {
        pinUpBound[i] = (double)cs.controller().motionAtAbs(i).maxPosCount() / cs.controller().motionAtAbs(i).pos2countRatio() - safetyOffset;
        pinLowBound[i] = (double)cs.controller().motionAtAbs(i).minPosCount() / cs.controller().motionAtAbs(i).pos2countRatio() + safetyOffset;
    }
#endif

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    double targetPeb[6]{ 0 };
    targetPeb[0] = param.x;
    targetPeb[1] = param.y;
    targetPeb[2] = param.z;
    targetPeb[3] = param.pitch;
    targetPeb[4] = param.yaw;
    targetPeb[5] = param.roll;
    //行程检测
    if (param.count == 0)
    {
        double targetPin[18]{ 0 };
        robot.SetPeb(targetPeb, beginMak, "123");
        robot.SetPee(Pee, beginMak);
        robot.GetPin(targetPin);
        bool isOverStroke{ false };
        for (int i = 0; i < 18; ++i)
        {
            if (targetPin[i] > pinUpBound[i])
            {
                rt_printf("Motor %i's target position %f is larger than its Up Bound %f.\n", i, targetPin[i], pinUpBound[i]);
                isOverStroke = true;
            }
            if (targetPin[i] < pinLowBound[i])
            {
                rt_printf("Motor %i's target position %f is smaller than its Low Bound %f.\n", i, targetPin[i], pinLowBound[i]);
                isOverStroke = true;
            }
        }
        if (isOverStroke)
        {
            return 0;
        }
    }

    const double s = -0.5 * cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1. 

    for (int i = 0; i < 6; ++i)
    {
        Peb[i] = targetPeb[i] * s;
    }

    robot.SetPeb(Peb, beginMak, "123");
    robot.SetPee(Pee, beginMak);

    return param.totalCount - param.count - 1;
}
