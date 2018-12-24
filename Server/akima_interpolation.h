#pragma once
#include <iostream>
#include <cmath>

double akima_interpolation(double t_i, const double *t, const double *y, int n);
int find_interval_id(double t_i, const double *t, int n);
