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
    static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

    const double safetyOffset{ 0 };
    double pinUpBound[18]{ 0 };
    double pinLowBound[18]{ 0 };

    for (int i = 0; i < 18; ++i)
    {
        pinUpBound[i] = (double)cs.controller().motionAtAbs(i).maxPosCount() / cs.controller().motionAtAbs(i).pos2countRatio() - safetyOffset;
        pinLowBound[i] = (double)cs.controller().motionAtAbs(i).minPosCount() / cs.controller().motionAtAbs(i).pos2countRatio() + safetyOffset;
    }

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

        rt_printf("pinUpBound:\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n",
            pinUpBound[0], pinUpBound[1], pinUpBound[2], pinUpBound[3], pinUpBound[4], pinUpBound[5], 
            pinUpBound[6], pinUpBound[7], pinUpBound[8], pinUpBound[9], pinUpBound[10], pinUpBound[11], 
            pinUpBound[12], pinUpBound[13], pinUpBound[14], pinUpBound[15], pinUpBound[16], pinUpBound[17]);
        rt_printf("pinLowBound:\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n",
            pinLowBound[0], pinLowBound[1], pinLowBound[2], pinLowBound[3], pinLowBound[4], pinLowBound[5],
            pinLowBound[6], pinLowBound[7], pinLowBound[8], pinLowBound[9], pinLowBound[10], pinLowBound[11],
            pinLowBound[12], pinLowBound[13], pinLowBound[14], pinLowBound[15], pinLowBound[16], pinLowBound[17]);
        rt_printf("targetPin:\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n",
            targetPin[0], targetPin[1], targetPin[2], targetPin[3], targetPin[4], targetPin[5], 
            targetPin[6], targetPin[7], targetPin[8], targetPin[9], targetPin[10], targetPin[11], 
            targetPin[12], targetPin[13], targetPin[14], targetPin[15], targetPin[16], targetPin[17]);
 
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
            robot.SetPeb(Peb, beginMak, "123");
            robot.SetPee(Pee, beginMak);
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
