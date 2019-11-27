#include "adjust_pee_2.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto adjustPeeParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    apParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
		if (i.first == "h")
		{
			param.h = std::stod(i.second);
		}
	}

    msg.copyStruct(param);
}

auto adjustPeeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const apParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

	if (param.count%param.totalCount == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetPee(beginPee, beginMak);
    }

	int period_count = param.count%param.totalCount;
	const double s = -0.5 * cos(PI * (period_count + 1) / param.totalCount) + 0.5; //s从0到1. 
    const int leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 3 : 0;

	double Peb[6], Pee[18];
	std::fill(Peb, Peb + 6, 0);
	std::copy(beginPee, beginPee + 18, Pee);
	for (int i = leg_begin_id; i < 18; i += 6)
    {
        Pee[i] = beginPee[i] * (1 + std::cos(PI*s)) / 2 + param.recoverPee[i] * (1 - std::cos(PI*s)) / 2; // x
        Pee[i + 1] += param.h * std::sin(PI*s); // y
        Pee[i + 2] = beginPee[i + 2] * (1 + std::cos(PI*s)) / 2 + param.recoverPee[i + 2] * (1 - std::cos(PI*s)) / 2; // z
    }

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

	return 2 * param.totalCount - param.count - 1;
}
