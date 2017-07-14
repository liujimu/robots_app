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
    const double safetyOffset{ 0.005 };
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
	const double s = -0.5 * cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1. 

	for (int i = 0; i < 6; ++i)
	{
		Peb[i] = targetPeb[i] * s;
	}

    robot.SetPeb(Peb, beginMak, "123");
    robot.SetPee(Pee, beginMak);
    //行程检测
    double Pin[18]{ 0 };
    robot.GetPin(Pin);
    bool isOverBound{ false };
    for (int i = 0; i < 18; ++i)
    {
        if (Pin[i] > pinUpBound[i] || Pin[i] < pinLowBound)
        {
            rt_printf("Getting close to the travel limits.");
            isOverBound = true;
        }
    }

    return param.totalCount - param.count - 1;
}
