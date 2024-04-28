#pragma once
#include <cmath>

#define PI 3.14159265358979

/**
 * @brief 角度转弧度
*/
double DEG2R(double deg){
    return deg / 180.0 * PI;
}

/**
 * @brief 弧度转角度
*/
double R2DEG(double r){
    return r / PI * 180.0;
}

/**
 * @brief 死区过滤
*/
double DB(double axisVal) {
    if (axisVal < 0.06 && axisVal > -0.06){
        axisVal = 0;
    }
    return axisVal;
}

/**
 * @brief 峰值滤波
*/
double Cap(double value, double peak) {
    if (value < -peak){
        return -peak;
    }
    if (value > +peak){
        return +peak;
    }
    return value;
}

/**
 * @brief S曲线函数
*/
double S(double value){
    return 1.0 / (1.0 + exp(-8.0 * value)) - 0.5;
}
