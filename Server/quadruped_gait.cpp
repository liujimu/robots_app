#include "quadruped_gait.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto quadrupedGaitParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    qgParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.total_count = std::stoi(i.second);
        }
        else if (i.first == "bodyUp")
        {
            param.body_up = std::stod(i.second);
        }
        else if (i.first == "bodyBack")
        {
            param.body_back = std::stod(i.second);
        }
        else if (i.first == "bodyPitch")
        {
            param.body_pitch = std::stod(i.second) / 180 * PI;
        }
        //else if (i.first == "frontFootUpwardOffset")
        //{
        //    param.front_foot_upward_offset = std::stod(i.second);
        //}
        else if (i.first == "frontFootInnerOffset")
        {
            param.front_foot_inner_offset = std::stod(i.second);
        }
        //else if (i.first == "frontFootUpwardOffset2")
        //{
        //    param.front_foot_upward_offset_2 = std::stod(i.second);
        //}
        //else if (i.first == "frontFootBackwardOffset2")
        //{
        //    param.front_foot_backward_offset_2 = std::stod(i.second);
        //}
        else if (i.first == "middleFootForwardOffset")
        {
            param.middle_foot_forward_offset = std::stod(i.second);
        }
        //else if (i.first == "middleFootInnerOffset")
        //{
        //    param.middle_foot_inner_offset = std::stod(i.second);
        //}
        else if (i.first == "rearFootOuterOffset")
        {
            param.rear_foot_outer_offset = std::stod(i.second);
        }
        //else if (i.first == "bodyDown")
        //{
        //    param.body_down = std::stod(i.second);
        //}
        //else if (i.first == "bodySidesway")
        //{
        //    param.body_sidesway = std::stod(i.second);
        //}
        else if (i.first == "walkStepLength")
        {
            param.walk_step_length = std::stod(i.second);
        }
        else if (i.first == "walkStepHeight")
        {
            param.walk_step_height = std::stod(i.second);
        }
        else if (i.first == "n")
        {
            param.n = std::stoi(i.second);
        }
    }

    msg.copyStruct(param);
}

auto quadrupedGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const qgParam &>(param_in);

    //初始化静态变量
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double lastStepPeb[6]; //上一步结束时身体位姿
    static double lastStepPee[18]; //上一步结束时足尖位置
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        std::fill(lastStepPeb, lastStepPeb + 6, 0);
        std::copy(beginPee, beginPee + 18, lastStepPee);
    }

    int total_count = param.total_count;
    int n = param.n;
    int step_id = param.count / total_count; //step_id从0到7
    int period_count = param.count % total_count;
    double s = -(PI / 2) * cos(PI * (period_count + 1) / total_count) + PI / 2; //s从0到PI. 

    double Peb[6], Pee[18];
    std::copy(lastStepPeb, lastStepPeb + 6, Peb);
    std::copy(lastStepPee, lastStepPee + 18, Pee);

    //前两步：调整后腿
    if (step_id < 2)
    {
        int leg_id = 3 * (step_id % 2) + 2;
        double dx = -param.rear_foot_outer_offset * pow(-1, leg_id); //leg_id==2,dx<0; leg_id==5,dx>0
        Pee[3 * leg_id] += dx * (1 - std::cos(s)) / 2;
        Pee[3 * leg_id + 1] += param.walk_step_height * std::sin(s);
        if (period_count == total_count - 1)
        {
            lastStepPee[3 * leg_id] += dx;
        }
    }

    //第三步：迈中间腿
    else if (step_id == 2)
    {
        Peb[1] += param.body_up*(1 - std::cos(s)) / 2;
        for (int i = 0; i < 18; i += 9)
        {
            double dx = param.middle_foot_inner_offset * pow(-1, i); //i==0,+; i==9,-
            Pee[i + 3] += dx  * (1 - std::cos(s)) / 2;
            Pee[i + 4] += param.walk_step_height * std::sin(s);
            Pee[i + 5] -= param.middle_foot_forward_offset * (1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPeb[1] += param.body_up;
            lastStepPee[3] += param.middle_foot_inner_offset;
            lastStepPee[5] -= param.middle_foot_forward_offset;
            lastStepPee[12] -= param.middle_foot_inner_offset;
            lastStepPee[14] -= param.middle_foot_forward_offset;
        }
    }

    //第四步：前腿夹取物块
    else if (step_id == 3)
    {
        //规划前腿
        for (int i = 0; i < 18; i += 9)
        {
            double dx = param.front_foot_inner_offset * pow(-1, i); //i==0,+; i==9,-
            Pee[i] += dx *(1 - std::cos(s)) / 2;
            Pee[i + 1] += param.front_foot_upward_offset *(1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPee[0] += param.front_foot_inner_offset;
            lastStepPee[1] += param.front_foot_upward_offset;
            lastStepPee[9] -= param.front_foot_inner_offset;
            lastStepPee[10] += param.front_foot_upward_offset;
        }
    }

    //第五步：调整身体，抬前腿
    else if (step_id == 4)
    {
        //规划身体
        Peb[2] += param.body_back*(1 - std::cos(s)) / 2;
        Peb[4] += param.body_pitch*(1 - std::cos(s)) / 2;
        //规划前腿
        for (int i = 0; i < 18; i += 9)
        {
            Pee[i + 1] += param.front_foot_upward_offset_2 *(1 - std::cos(s)) / 2;
            Pee[i + 2] += param.front_foot_backward_offset_2*(1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPeb[2] += param.body_back;
            lastStepPeb[4] += param.body_pitch;
            lastStepPee[1] += param.front_foot_upward_offset_2;
            lastStepPee[2] += param.front_foot_backward_offset_2;
            lastStepPee[10] += param.front_foot_upward_offset_2;
            lastStepPee[11] += param.front_foot_backward_offset_2;
        }
    }

    //第六步：行走
    else if (step_id < 5 + 8 * n)
    {
        const int step_order[4]{ 2,1,5,4 };
        int walk_step_id = (step_id - 5) % 8; //walk_step_id从0到7
        int leg_id = step_order[walk_step_id / 2]; //确定当前迈腿序号        
        //规划身体位置
        if (walk_step_id % 2 == 1)
        {
            double dx, dy, dz;
            switch (walk_step_id)
            {
            case 1:
                dx = param.body_sidesway;
                dy = -param.body_down;
                dz = 0.0;
                break;
            case 3:
                dx = -param.body_sidesway;
                dy = 0;
                dz = -param.walk_step_length / 2;
                break;
            case 5:
                dx = -param.body_sidesway;
                dy = 0;
                dz = 0;
                break;
            //case 6:
            //    dx = 0;
            //    dy = param.body_down;
            //    dz = 0;
            //    break;
            case 7:
                dx = param.body_sidesway;
                dy = param.body_down;
                dz = -param.walk_step_length / 2;
                break;
            default:
                dx = 0;
                dy = 0;
                dz = 0;
                break;
            }
            Peb[0] += dx * (1 - std::cos(s)) / 2;
            Peb[1] += dy * (1 - std::cos(s)) / 2;
            Peb[2] += dz * (1 - std::cos(s)) / 2;
            //规划两条前腿
            for (int i = 0; i < 6; i += 3)
            {
                Pee[3 * i] += dx * (1 - std::cos(s)) / 2;
                Pee[3 * i + 1] += dy * (1 - std::cos(s)) / 2;
                Pee[3 * i + 2] += dz * (1 - std::cos(s)) / 2;
            }
            if (period_count == param.total_count - 1)
            {
                lastStepPeb[0] += dx;
                lastStepPeb[1] += dy;
                lastStepPeb[2] += dz;
                for (int i = 0; i < 6; i += 3)
                {
                    lastStepPee[3 * i] += dx;
                    lastStepPee[3 * i + 1] += dy;
                    lastStepPee[3 * i + 2] += dz;
                }
            }
        }
        //规划足尖位置
        if (walk_step_id % 2 == 0)
        {
            Pee[3 * leg_id + 1] += param.walk_step_height * std::sin(s);
            Pee[3 * leg_id + 2] -= param.walk_step_length * (1 - std::cos(s)) / 2;
            if (period_count == total_count - 1)
            {
                lastStepPee[3 * leg_id + 2] -= param.walk_step_length;
            }
        }
    }

    //第七步：恢复身体，前腿放下物块
    else if (step_id == 5 + 8 * n)
    {
        //规划身体
        Peb[2] -= param.body_back*(1 - std::cos(s)) / 2;
        Peb[4] -= param.body_pitch*(1 - std::cos(s)) / 2;
        //规划前腿
        for (int i = 0; i < 18; i += 9)
        {
            Pee[i + 1] -= param.front_foot_upward_offset_2 *(1 - std::cos(s)) / 2;
            Pee[i + 2] -= param.front_foot_backward_offset_2*(1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPeb[2] -= param.body_back;
            lastStepPeb[4] -= param.body_pitch;
            lastStepPee[1] -= param.front_foot_upward_offset_2;
            lastStepPee[2] -= param.front_foot_backward_offset_2;
            lastStepPee[10] -= param.front_foot_upward_offset_2;
            lastStepPee[11] -= param.front_foot_backward_offset_2;
        }
    }

    //第八步：前腿松开物块并落地
    else if (step_id == 6 + 8 * n)
    {
        //规划前腿
        for (int i = 0; i < 18; i += 9)
        {
            double dx = param.front_foot_inner_offset * pow(-1, i); //i==0,+; i==9,-
            Pee[i] -= dx *(1 - std::cos(s)) / 2;
            Pee[i + 1] -= param.front_foot_upward_offset *(1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPee[0] -= param.front_foot_inner_offset;
            lastStepPee[1] -= param.front_foot_upward_offset;
            lastStepPee[9] += param.front_foot_inner_offset;
            lastStepPee[10] -= param.front_foot_upward_offset;
        }
    }

    //第九步：恢复中间腿
    else if (step_id == 7 + 8 * n)
    {
        Peb[1] -= param.body_up*(1 - std::cos(s)) / 2;
        for (int i = 0; i < 18; i += 9)
        {
            double dx = param.middle_foot_inner_offset * pow(-1, i); //i==0,+; i==9,-
            Pee[i + 3] -= dx  * (1 - std::cos(s)) / 2;
            Pee[i + 4] += param.walk_step_height * std::sin(s);
            Pee[i + 5] += param.middle_foot_forward_offset * (1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPeb[1] -= param.body_up;
            lastStepPee[3] -= param.middle_foot_inner_offset;
            lastStepPee[5] += param.middle_foot_forward_offset;
            lastStepPee[12] += param.middle_foot_inner_offset;
            lastStepPee[14] += param.middle_foot_forward_offset;
        }
    }

    //最后两步：调整后腿
    else if (step_id < 10 + 8 * n)
    {
        int leg_id = 3 * (step_id % 2) + 2;
        double dx = -param.rear_foot_outer_offset * pow(-1, leg_id); //leg_id==2,dx<0; leg_id==5,dx>0
        Pee[3 * leg_id] -= dx * (1 - std::cos(s)) / 2;
        Pee[3 * leg_id + 1] += param.walk_step_height * std::sin(s);
        if (period_count == total_count - 1)
        {
            lastStepPee[3 * leg_id] -= dx;
        }
    }

    if (period_count == total_count - 1)
    {
        rt_printf("count: %d\n", param.count);
        rt_printf("Peb: %f %f %f %f %f %f\n",Peb[0],Peb[1],Peb[2],Peb[3],Peb[4],Peb[5]);
        rt_printf("Pee: %f %f %f %f %f %f\n", Pee[1], Pee[4], Pee[7], Pee[10], Pee[13], Pee[16]);
    }

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return (10 + 8 * n) * param.total_count - param.count - 1;
}