#include "recover2.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto recover2Parse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    rc2Param param;

    param.if_check_pos_min = false;
    param.if_check_pos_max = false;

    for (auto &i : params)
    {
        if (i.first == "all")
        {
            std::fill_n(param.active_leg, 6, true);
        }
        else if (i.first == "first")
        {
            param.active_leg[0] = true;
            param.active_leg[1] = false;
            param.active_leg[2] = true;
            param.active_leg[3] = false;
            param.active_leg[4] = true;
            param.active_leg[5] = false;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 0, 3, true);
            std::fill_n(param.active_motor + 6, 3, true);
            std::fill_n(param.active_motor + 12, 3, true);
        }
        else if (i.first == "second")
        {
            param.active_leg[0] = false;
            param.active_leg[1] = true;
            param.active_leg[2] = false;
            param.active_leg[3] = true;
            param.active_leg[4] = false;
            param.active_leg[5] = true;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 3, 3, true);
            std::fill_n(param.active_motor + 9, 3, true);
            std::fill_n(param.active_motor + 15, 3, true);
        }
        else if (i.first == "leg")
        {
            auto leg_id = std::stoi(i.second);

            if (leg_id < 0 || leg_id>5)throw std::runtime_error("invalide param in parseRecover func");

            std::fill_n(param.active_leg, 6, false);
            param.active_leg[leg_id] = true;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + leg_id * 3, 3, true);
        }
        else if (i.first == "stepNumber")
        {
            param.stepNumber = std::stoi(i.second);
        }
        else if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "margin_offset")
        {
            param.margin_offset = std::stod(i.second);
        }
    }

    msg.copyStruct(param);
}

auto recover2Gait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const rc2Param &>(param_in);

    //行程检测
    static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

    static double beginPin[18], beginPee[18], targetPin[18];

    if (param.count == 0)
    {
        std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);
        robot.GetPee(beginPee, robot.body());

        const double pe[6]{ 0 };
        robot.SetPeb(pe);
        robot.SetPee(param.targetPee[param.stepNumber]);

        robot.GetPin(targetPin);
        robot.SetPee(beginPee, robot.body());
    }

    const double s = -0.5 * cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1. 

    for (int i = 0; i < 6; ++i)
    {
        if (param.active_leg[i])
        {
            for (int j = 0; j < 3; ++j)
            {
                robot.motionPool().at(i * 3 + j).setMotPos(beginPin[i * 3 + j] * (1 - s) + targetPin[i * 3 + j] * s);
            }
        }
    }
    if (param.count == param.totalCount - 1)
    {
        robot.SetPee(param.targetPee[param.stepNumber], robot.body());
    }

    // recover 自己做检查 // 
    for (int i = 0; i<18; ++i)
    {
        if (param.active_motor[i] && (param.last_motion_raw_data->at(i).cmd == aris::control::EthercatMotion::RUN))
        {

            if (param.motion_raw_data->at(i).target_pos >(cs.controller().motionAtAbs(i).maxPosCount() + param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
            {
                rt_printf("Motor %i's target position is bigger than its MAX permitted value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover2 failed\n");
                return 0;
            }
            if (param.motion_raw_data->at(i).target_pos < (cs.controller().motionAtAbs(i).minPosCount() - param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
            {
                rt_printf("Motor %i's target position is smaller than its MIN permitted value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover2 failed\n");
                return 0;
            }
        }
    }

    return param.totalCount - param.count - 1;
}
