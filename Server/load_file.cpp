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

//记录数据
LogParam LoadFile::logParam;
aris::control::Pipe<LogParam> LoadFile::lfPipe(true);
std::thread LoadFile::lfThread;

void LoadFile::loadFileParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    lfParam param;

    double data[DATA_SIZE][3];
    aris::dynamic::dlmread("../../data/data.txt", *data);
    for (int i = 0; i < DATA_SIZE; ++i)
    {
        data_t[i] = data[i][0];
        data_y[i] = 0.001 * data[i][1];
        data_theta[i] = PI / 180 * data[i][2];
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

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    const double pe_tool_init[6]{ 0, 0.6042, 0, 0, 0, 0 };
    double pm_tool_init[16];
    aris::dynamic::s_pe2pm(pe_tool_init, pm_tool_init, "123");
    
    double ti = 0.001*(param.count + 1);
    double yi = akima_interpolation(ti, data_t, data_y, DATA_SIZE);
    double thetai = akima_interpolation(ti, data_t, data_theta, DATA_SIZE);

    double pe_tool_rel[6]{ 0, 0, 0, 0, 0, 0 };
    if (param.dir < 3)
    {
        pe_tool_rel[param.dir] = yi;
    }
    else
    {
        pe_tool_rel[param.dir] = thetai;
    }
    double pm_tool_rel[16];
    aris::dynamic::s_pe2pm(pe_tool_rel, pm_tool_rel, "123");

    double pm_tool_cur[16];
    aris::dynamic::s_pm_dot_pm(pm_tool_init, pm_tool_rel, pm_tool_cur);

    const double pe_tool2body[6]{ 0, -0.6042, 0, 0, 0, 0 };
    double pm_tool2body[16];
    aris::dynamic::s_pe2pm(pe_tool2body, pm_tool2body, "123");

    double pm_body[16];
    aris::dynamic::s_pm_dot_pm(pm_tool_cur, pm_tool2body, pm_body);
    
    //for test
    if (param.count % 1000 == 0)
    {
        double peb[6];
        aris::dynamic::s_pm2pe(pm_body, peb, "123");
        rt_printf("count: %d\n", param.count);
        rt_printf("pe_tool_init: %f, %f, %f\n", pm_tool_init[3], pm_tool_init[7], pm_tool_init[11]);
        rt_printf("pe_tool_rel: %f, %f, %f\n", pm_tool_rel[3], pm_tool_rel[7], pm_tool_rel[11]);
        rt_printf("pm_tool_cur: %f, %f, %f\n", pm_tool_cur[3], pm_tool_cur[7], pm_tool_cur[11]);
        rt_printf("pm_tool2body: %f, %f, %f\n", pm_tool2body[3], pm_tool2body[7], pm_tool2body[11]);
        rt_printf("peb: %f, %f, %f, %f, %f, %f\n", peb[0], peb[1], peb[2], peb[3], peb[4], peb[5]);
    }

    robot.SetPmb(pm_body, beginMak);
    robot.SetPee(beginPee);

    //记录力信号数据并发送到pipe
    std::copy_n(param.ruicong_data->at(0).force[6].fce, 6, logParam.fce);
    double peb[6];
    robot.GetPeb(peb, beginMak, "123");
    std::copy_n(peb, 6, logParam.peb);
    lfPipe.sendToNrt(logParam);

    return param.totalCount - param.count - 1;
}

void LoadFile::recordData()
{
    lfThread = std::thread([&]()
    {
        struct LogParam param;
        static std::fstream fileGait;
        std::string name = aris::core::logFileName();
        name.replace(name.rfind("log.txt"), std::strlen("forceData.txt"), "forceData.txt");
        fileGait.open(name.c_str(), std::ios::out | std::ios::trunc);

        long long count = -1;
        while (1)
        {
            lfPipe.recvInNrt(param);

            fileGait << ++count << " ";
            for (int i = 0; i<6; i++)
            {
                fileGait << param.fce[i] << "  ";
            }
            fileGait << std::endl;
        }
        fileGait.close();
    });
}
