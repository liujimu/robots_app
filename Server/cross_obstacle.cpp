#include "cross_obstacle.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto crossObstacleParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    coParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
		if (i.first == "n")
		{
			param.n = std::stoi(i.second);
		}
		if (i.first == "d")
		{
			param.d = std::stod(i.second);
		}
		if (i.first == "h")
		{
			param.h = std::stod(i.second);
		}
		if (i.first == "y")
		{
			param.y = std::stod(i.second);
		}
	}

    msg.copyStruct(param);
}

auto crossObstacleGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const coParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double targetPeb[6];

	if (param.count%param.totalCount == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetPee(beginPee, beginMak);
		std::fill(targetPeb, targetPeb + 6, 0);
    }

    double d{ 0 };
    double h{ 0 };
    if ((param.count / param.totalCount) == 0)//第一步
    {
		d = param.d / std::fabs(param.d) * 0.2;
		h = 0.05;
        targetPeb[0] = d / 2;
        targetPeb[1] = param.y;
    }
    else if ((param.count / param.totalCount) == (2 * param.n - 1))//最后一步
    {
        d = param.d / std::fabs(param.d) * 0.2;
		h = 0.05;
        targetPeb[0] = d / 2;
		targetPeb[1] = -param.y;
    }
    else
    {
		if ((param.count / param.totalCount) < 3 || (param.count / param.totalCount) > (2 * param.n - 4))
		{
			d = param.d;
		}
		else
		{
			d = param.d / std::fabs(param.d) * (1.4 - std::fabs(param.d)) / (param.n - 3);
		}
		h = param.h;
        targetPeb[0] = d / 2;
    }

	int leg_begin_id;
	if (param.d > 0)
	{
		leg_begin_id = (param.count / param.totalCount) % 2 == 0 ? 3 : 0;
	}
	else
	{
		leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 3 : 0;
	}
	int period_count = param.count%param.totalCount;
	const double s = -0.5 * cos(PI * (period_count + 1) / param.totalCount) + 0.5; //s从0到1. 

	double Peb[6], Pee[18];
	std::fill(Peb, Peb + 6, 0);
	std::copy(beginPee, beginPee + 18, Pee);
	for (int i = leg_begin_id; i < 18; i += 6)
    {
        Pee[i] += d*(1 - std::cos(PI*s)) / 2;
        Pee[i + 1] += h*std::sin(PI*s);
    }
	for (int i = 0; i < 3; ++i)
	{
		Peb[i] = targetPeb[i] * s;
	}

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

	return 2 * param.n * param.totalCount - param.count - 1;
}
