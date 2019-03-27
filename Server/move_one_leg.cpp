#include "move_one_leg.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto moveOneLegParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    molParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "d")
        {
            param.d = std::stod(i.second);
        }
        else if (i.first == "h")
        {
            param.h = std::stod(i.second);
        }
        else if (i.first == "l")
        {
            param.legID = std::stoi(i.second);
        }
    }

    msg.copyStruct(param);
}

auto moveOneLegGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const molParam &>(param_in);

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

	const double s = -0.5 * cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1. 

    const int i = param.legID;
    const double d = param.d;
    const double h = param.h;

    Pee[i * 3 + 1] += h*std::sin(PI*s);
    Pee[i * 3 + 2] += -d*(1 - std::cos(PI*s)) / 2;

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return param.totalCount - param.count - 1;
}
