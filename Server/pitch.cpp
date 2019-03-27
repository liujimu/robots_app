#include "pitch.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto pitchParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    pitchParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "y")
        {
            param.y = stod(i.second);
        }
        else if (i.first == "z")
        {
            param.z = stod(i.second);
        }
        else if (i.first == "a")
        {
            param.a = stod(i.second) / 180 * PI;
        }
    }

    msg.copyStruct(param);
}

auto pitchGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const pitchParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double lastPeb[6];
    static bool isWalking{ false };
    static int beginCount{ 0 };

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double Peb[6], Pee[18];
    
    if (param.count < 2 * param.totalCount)
    {
        std::fill(Peb, Peb + 6, 0);
        std::copy(beginPee, beginPee + 18, Pee);

        int period_count = param.count%param.totalCount;
        const double s = -0.5 * cos(PI * (period_count + 1) / param.totalCount) + 0.5; //s从0到1. 
        const int leg_begin_id = (param.count / param.totalCount) % 2 == 0 ? 3 : 0;
        for (int i = leg_begin_id; i < 18; i += 6)
        {
            Pee[i] = beginPee[i] * (1 - s) + param.prePee[i] * s; //x
            Pee[i + 2] = beginPee[i + 2] * (1 - s) + param.prePee[i + 2] * s; //y
            Pee[i + 1] += param.h * std::sin( PI * s ); //z
        }
    }
    else if (param.count < 3 * param.totalCount)
    {
        std::copy(param.prePee, param.prePee + 18, Pee);

        const double s = -0.5 * cos(PI * (param.count - 2 * param.totalCount + 1) / param.totalCount) + 0.5;//s 从0到1.
        Peb[0] = 0;
        Peb[1] = lastPeb[1] * (1 - s) + param.y * s;
        Peb[2] = lastPeb[2] * (1 - s) + param.z * s;
        Peb[3] = 0;
        Peb[4] = lastPeb[4] * (1 - s) + param.a * s;
        Peb[5] = 0;
    }
    else
    {
        if (PitchState::getState().isStopping())
        {

        }

    }


    const double s = -0.5 * cos(PI * (param.count - beginCount + 1) / param.totalCount) + 0.5;//s 从0到1.

	Peb[0] = 0;
    Peb[1] = lastPeb[1] * (1 - s) + param.y * s;
	Peb[2] = lastPeb[2] * (1 - s) + param.z * s;
	Peb[3] = 0;
	Peb[4] = lastPeb[4] * (1 - s) + param.a * s;
	Peb[5] = 0;

	robot.SetPeb(Peb, beginMak);
	robot.SetPee(Pee, beginMak);

    if (PitchState::getState().isStopping() && (param.count - beginCount + 1 == 3 * param.totalCount))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
