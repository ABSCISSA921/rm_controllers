//
// Created by wk on 2025/12/26.
//

#include "bipedal_wheel_controller/vmc/leg_conv_t.h"
#include "bipedal_wheel_controller/vmc/leg_params.h"
#include <math.h>

void leg_conv_t(double T1, double T2, double phi1, double phi4, double F[2])
{
  double YD, YB, XD, XB, lBD, A0, B0, C0, XC, YC;
  double phi2, phi3;
  double L0, phi0;

  YD = LEG_L4 * sin(phi4);
  YB = LEG_L1 * sin(phi1);
  XD = LEG_L5 + LEG_L4 * cos(phi4);
  XB = LEG_L1 * cos(phi1);
  lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
  A0 = 2 * LEG_L2 * (XD - XB);
  B0 = 2 * LEG_L2 * (YD - YB);
  C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
  phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
  phi3 = atan2(YB - YD + LEG_L2 * sin(phi2), XB - XD + LEG_L2 * cos(phi2));
  XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
  YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
  L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
  phi0 = atan2(YC, XC - LEG_L5 / 2);

  double t11_tmp = (T2 * cos(phi0 - phi3)) / (LEG_L4 * sin(phi3 - phi4));
  double t12_tmp = (T1 * cos(phi0 - phi2)) / (LEG_L1 * sin(phi1 - phi2));
  F[0] = t11_tmp - t12_tmp;
  F[1] = L0 * (t12_tmp - t11_tmp);
}