#include "load_file.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

double LoadFile::data_t[DATA_SIZE];
double LoadFile::data_y[DATA_SIZE];
double LoadFile::data_theta[DATA_SIZE];

void LoadFile::loadFileParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    lfParam param;

    double data[DATA_SIZE][3];
    aris::dynamic::dlmread("D:\\Lab\\src\\Robots\\test\\data\\data.txt", *data);
    for (int i = 0; i < DATA_SIZE; ++i)
    {
        data_t[i] = data[i][0];
        data_y[i] = data[i][1];
        data_theta[i] = data[i][2];
    }
    std::cout << "Data loaded." << std::endl;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "y")
        {
            param.dir = 1;
        }
        else if (i.first == "u")
        {
            param.dir = 3;
        }
        else if (i.first == "w")
        {
            param.dir = 5;
        }
    }

    msg.copyStruct(param);
}

int LoadFile::loadFileGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const lfParam &>(param_in);

    //≥ı ºªØ
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    const double pe_tool_init[6]{ 0,0.6042,0,0,0,0 };
    double pm_tool_init[16];
    aris::dynamic::s_pe2pm(pe_tool_init, pm_tool_init);
    
    double ti = 0.001*(param.count + 1);
    double yi = akima_interpolation(ti, data_t, data_y, DATA_SIZE);
    double thetai = akima_interpolation(ti, data_t, data_theta, DATA_SIZE);

    double pe_tool_rel[6]{ 0 };
    if (param.dir < 3)
    {
        pe_tool_rel[1] = yi;
    }
    else
    {
        pe_tool_rel[3] = thetai;
    }
    double pm_tool_rel[16];
    aris::dynamic::s_pe2pm(pe_tool_init, pm_tool_rel);

    double pm_tool_cur[16];
    aris::dynamic::s_pm_dot_pm(pm_tool_init, pm_tool_rel, pm_tool_cur);

    const double pe_tool2body[6]{ 0,-0.6042,0,0,0,0 };
    double pm_tool2body[16];
    aris::dynamic::s_pe2pm(pe_tool2body, pm_tool2body);

    double pm_body[16];
    aris::dynamic::s_pm_dot_pm(pm_tool_cur, pm_tool2body, pm_body);

    robot.SetPmb(pm_body, beginMak);
    robot.SetPee(beginPee);

    return param.totalCount - param.count - 1;
}
