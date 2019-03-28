#include "sine_motion.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto sineMotionParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    smParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "y")
        {
            param.dir = 1;
            param.amp = 0.05;
        }
        else if (i.first == "u")
        {
            param.dir = 3;
            param.amp = PI / 180 * 10;
        }
        else if (i.first == "w")
        {
            param.dir = 5;
            param.amp = PI / 180 * 10;
        }
        else if (i.first == "cycle")
        {
            param.cycle = std::stod(i.second);
        }
        else if (i.first == "radius")
        {
            param.radius = std::stod(i.second);
        }
    }

    msg.copyStruct(param);
}

auto sineMotionGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const smParam &>(param_in);

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

    const int preparation_count{ 1500 };
    if (param.count <= preparation_count)
    {
        const double s = -0.5 * cos(PI * (param.count + 1) / preparation_count) + 0.5;//s 从0到1.
        switch (param.dir)
        {
        case 1:
            Peb[1] = param.amp*s;
            break;
        case 3:
            Peb[3] = param.amp*s;
            Peb[1] = param.radius*(1 - std::cos(Peb[3]));
            Peb[2] = -param.radius*std::sin(Peb[3]);
            break;
        case 5:
            Peb[5] = param.amp*s;
            Peb[0] = param.radius*std::sin(Peb[5]);
            Peb[1] = param.radius*(1 - std::cos(Peb[5]));
            break;
        default:
            break;
        }
    }
    else
    {
        const double t = 0.001*(param.count - preparation_count + 1);
        switch (param.dir)
        {
        case 1:
            Peb[1] = param.amp*std::cos(2 * PI / param.cycle*t);
            break;
        case 3:
            Peb[3] = param.amp*std::cos(2 * PI / param.cycle*t);
            Peb[1] = param.radius*(1 - std::cos(Peb[3]));
            Peb[2] = -param.radius*std::sin(Peb[3]);
            break;
        case 5:
            Peb[5] = param.amp*std::cos(2 * PI / param.cycle*t);
            Peb[0] = param.radius*std::sin(Peb[5]);
            Peb[1] = param.radius*(1 - std::cos(Peb[5]));
            break;
        default:
            break;
        }
    }

    robot.SetPeb(Peb, beginMak, "123");
    robot.SetPee(Pee, beginMak);

    return param.totalCount + preparation_count + int(param.cycle * 1000 / 4) - param.count - 1;
}
