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
        else if (i.first == "leg_id")
        {
            param.leg_id = std::stoi(i.second);
            if (param.leg_id < 0 || param.leg_id > 5)
                throw std::runtime_error("invalid leg_id");
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + param.leg_id * 3, 3, true);
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

    double Peb[6], legPee[3];
    //std::fill(Peb, Peb + 6, 0);
    std::copy_n(beginPee + param.leg_id * 3, 3, legPee);
    
    const double s = -0.5 * cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1. 
    const double disp[3]{ param.x, param.y, param.z };

    for (int i = 0; i < 3; ++i)
    {
        legPee[i] += disp[i]*(1 - std::cos(PI*s)) / 2;
    }

    robot.pLegs[param.leg_id]->SetPee(legPee);

    //test
    if (param.count == 0)
    {
        rt_printf("leg_id: %d\n", param.leg_id);
        rt_printf("legPee: %f, %f, %f\n", legPee[0], legPee[1], legPee[2]);
    }

    return param.totalCount - param.count - 1;
}
