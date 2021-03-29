#pragma once

void    ERP_GetData(int const w1, int const w2, int* const pAngle, int* const pIncrement);
double  ERP_AngleTo360deg(double angle);
int64_t ERP_getIncrement(int const w1, int const w2);
double  ERP_incrementTo360deg(int increment);
