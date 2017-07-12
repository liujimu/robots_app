#include "move_body_with_force.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto moveBodyWithForceParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    mbfParam param;

    for (auto &i : params)
    {
        if (i.first == "forceThreshold")
        {
            param.forceThreshold = std::stod(i.second);
        }
        else if (i.first == "forceMaximum")
        {
            param.forceMaximum = std::stod(i.second);
        }
    }
    MbfState::getState().isStopping() = false;
    msg.copyStruct(param);
}

auto moveBodyWithForceStopParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    MbfState::getState().isStopping() = true;
}

auto moveBodyWithForceGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const mbfParam &>(param_in);

    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    //阻抗控制参数
    double F[6]{ 0, 0, 0, 0, 0, 0 };
    double M[6]{ 1, 1, 1, 1, 1, 1 };
    double K[6]{ 0, 0, 0, 0, 0, 0 };
    double C[6]{ 30, 60, 30, 20, 20, 20 };
    double bodyAcc[4]{ 0 };
    static double bodyVel[4]{ 0 };
    static double bodyDisp[4]{ 0 };
    const double kClockPeriod{ 0.001 };

    //力传感器数据
    static double forceOffsetSum[6]{ 0 };
    double forceOffsetAvg[6]{ 0 };
    double realForceData[6]{ 0 };
    double forceInBody[6]{ 0 };
    double forceThreshold[6]{ 20, 20, 20, 10, 10, 10 }; //力传感器的触发阈值,单位N或Nm
    double forceMax[6]{ 100, 100, 100, 50, 50, 50 }; //力的上限
    for (int i = 0; i < 3; i++)
    {
        forceThreshold[i] = param.forceThreshold;
        forceMax[i] = param.forceMaximum;
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    //力传感器手动清零
    if (param.count < 100)
    {
        if(param.count == 0)
        {
            std::fill(forceOffsetSum, forceOffsetSum + 6, 0);

            std::fill(bodyDisp, bodyDisp + 4, 0);
            std::fill(bodyVel, bodyVel + 4, 0);
        }
        for (int i = 0; i < 6; i++)
        {
            forceOffsetSum[i] += 10 * param.ruicong_data->at(0).force[0].fce[i];
        }
    }
    else
    {
        for(int i = 0; i < 6; i++)
        {
            forceOffsetAvg[i] = forceOffsetSum[i] / 100;
            realForceData[i] = 10 * param.ruicong_data->at(0).force[0].fce[i] - forceOffsetAvg[i];
            //若力超过设定的最大值，则只取最大值
            if(std::fabs(realForceData[i]) > forceMax[i])
            {
                realForceData[i] = realForceData[i] / std::fabs(realForceData[i]) * forceMax[i];
            }
        }
        //转换到机器人身体坐标系
        aris::dynamic::s_f2f(*robot.forceSensorMak().prtPm(), realForceData, forceInBody);
        //显示力的初始值
        if(param.count == 100)
        {
            rt_printf("forceOffsetAvg: %f %f %f %f %f %f\n",
                      forceOffsetAvg[0], forceOffsetAvg[1], forceOffsetAvg[2],
                      forceOffsetAvg[3], forceOffsetAvg[4], forceOffsetAvg[5]);
        }

        //人手推动机器人身体移动
        double maxForce{ 0 };
        int index{ 0 };
        for (int i = 0; i < 3; i++)
        {
            if (std::fabs(forceInBody[i]) > maxForce)
            {
                maxForce = std::fabs(forceInBody[i]);
                index = i;
            }
        }
        if (maxForce > forceThreshold[index])
        {
            for (int i = 0; i < 3; i++)
            {
                F[i] = forceInBody[i] / forceMax[i];
            }
        }
	if (std::fabs(forceInBody[4]) > forceThreshold[4])
	{
	    F[3] = forceInBody[4] /forceMax[4];
	}

        for (int i = 0; i < 4; i++)
        {
            bodyAcc[i] = (F[i] - C[i] * bodyVel[i] - K[i] * bodyDisp[i]) / M[i];
            bodyVel[i] += bodyAcc[i] * kClockPeriod;
            bodyDisp[i] += bodyVel[i] * kClockPeriod;
        }

        std::copy(bodyDisp, bodyDisp + 4, Peb);
        robot.SetPeb(Peb, beginMak, "213");
        robot.SetPee(Pee, beginMak);
    }

    if (MbfState::getState().isStopping())
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
