#ifndef CONFIG_H_
#define CONFIG_H_

#define pi 3.1415926535898
 
namespace Conversions
{
    //Speed
    float MPH_TO_KPH = 1.609344;
    float KPH_TO_MPH = 1.0 / MPH_TO_KPH;
    float MS_TO_KPH = 3.6;
    float KPH_TO_MS = 1.0 / MS_TO_KPH;
    float MS_TO_MPH = MS_TO_KPH * KPH_TO_MPH;
    float MPH_TO_MS = MPH_TO_KPH * KPH_TO_MS;
    float MS_TO_KNOTS = 1.9438;
    float KNOTS_TO_MS = 1.0 / MS_TO_KNOTS;
    //Angle
    float DEG_TO_RAD = pi / 180;
    float RAD_TO_DEG = 1.0 / DEG_TO_RAD;
    //Mass
    float LB_TO_KG = 0.453592;
}

float RADAR_TO_CENTER = 2.7;   // (deprecated) RADAR is ~ 2.7m ahead from center of car
float RADAR_TO_CAMERA = 1.52;   // RADAR is ~ 1.5m ahead from center of mesh frame

namespace UIParams
{
    int lidar_x, lidar_y, lidar_zoom = 384, 960, 6;
    float lidar_car_x, lidar_car_y = lidar_x/2.0, lidar_y/1.1;
    float car_hwidth = 1.7272/2 * lidar_zoom;
    float car_front = 2.6924 * lidar_zoom;
    float car_back  = 1.8796 * lidar_zoom;
    int car_color = 110;
}

#endif //CONFIG_H_