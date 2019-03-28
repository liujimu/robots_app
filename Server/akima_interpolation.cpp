#include "akima_interpolation.h"

double akima_interpolation(double ti, const double *t, const double *y, int n)
{
    int id = find_interval_id(ti, t, n);

    double m1, m2, m3, m4, m5;
    m3 = (y[id + 1] - y[id]) / (t[id + 1] - t[id]);
    if (id < 2)
    {
        m4 = (y[id + 2] - y[id + 1]) / (t[id + 2] - t[id + 1]);
        m5 = (y[id + 3] - y[id + 2]) / (t[id + 3] - t[id + 2]);
        m2 = 2 * m3 - m4;
        m1 = 2 * m2 - m3;
    }
    else if (id > n - 4)
    {
        m1 = (y[id - 1] - y[id - 2]) / (t[id - 1] - t[id - 2]);
        m2 = (y[id] - y[id - 1]) / (t[id] - t[id - 1]);
        m4 = 2 * m3 - m2;
        m5 = 2 * m4 - m3;
    }
    else
    {
        m1 = (y[id - 1] - y[id - 2]) / (t[id - 1] - t[id - 2]);
        m2 = (y[id] - y[id - 1]) / (t[id] - t[id - 1]);
        m4 = (y[id + 2] - y[id + 1]) / (t[id + 2] - t[id + 1]);
        m5 = (y[id + 3] - y[id + 2]) / (t[id + 3] - t[id + 2]);
    }

    double k3, k4;
    if (m1 == m2 && m3 == m4)
    {
        k3 = (m2 + m3) / 2;
    }
    else
    {
        k3 = (std::fabs(m4 - m3)*m2 + std::fabs(m2 - m1)*m3) / (std::fabs(m4 - m3) + std::fabs(m2 - m1));
    }
    if (m2 == m3 && m4 == m5)
    {
        k4 = (m3 + m4) / 2;
    }
    else
    {
        k4 = (std::fabs(m5 - m4)*m3 + std::fabs(m3 - m2)*m4) / (std::fabs(m5 - m4) + std::fabs(m3 - m2));
    }

    double p0 = y[id];
    double p1 = k3;
    double p2 = (3 * (y[id + 1] - y[id]) / (t[id + 1] - t[id]) - 2 * k3 - k4) / (t[id + 1] - t[id]);
    double p3 = (k3 + k4 - 2 * (y[id + 1] - y[id]) / (t[id + 1] - t[id])) / std::pow(t[id + 1] - t[id], 2);

    double yi = p0 + p1*(ti - t[id]) + p2*std::pow(ti - t[id], 2) + p3*std::pow(ti - t[id], 3);

    return yi;
}

int find_interval_id(double ti, const double *t, int n)
{
    int i = 0;
    while (t[i] < ti)
    {
        ++i;
    }
    if (i < 1 || i >= n)
    {
        std::cout << "i = " << i << " ,out of range." << std::endl;
    }
    return --i;
}