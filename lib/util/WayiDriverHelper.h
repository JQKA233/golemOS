#pragma once
#include <cmath>
#include "Functions.h"

typedef struct
{
    double x;
    double y;
    double w;
}Vec_3d;

class WayiDriverHelper
{
private:
    const double kHighWheelNonLinearity = 0.65;
    const double kLowWheelNonLinearity = 0.5;
    bool isDeadBand = true;

    
public:
    WayiDriverHelper()
    {
    }

    ~WayiDriverHelper()
    {
    }
    
    
    double NonleanerControl(double input,bool isHighGear,bool isSuper = false){
        double wheelNonLinearity;
        if (!isHighGear) {
            // wheelNonLinearity = kHighWheelNonLinearity;
            // 下凸曲线
            input = 0.31083769050760857*sinh(0.5859152429745125*sinh(1.8849555921538759*input));
        }
        else if(isSuper){
            // 极致下凸曲线
            input = 0.118745*sinh(0.335743*sinh(2.82743*input));
        }
        else {
            wheelNonLinearity = kLowWheelNonLinearity;
            double denominator = sin(PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            input = sin(PI / 2.0 * wheelNonLinearity * input) / denominator;
            input = sin(PI / 2.0 * wheelNonLinearity * input) / denominator;
            input = sin(PI / 2.0 * wheelNonLinearity * input) / denominator;
            
        }
        return input;
    }


    
    double deathJudge1D(double s){
        if((s>0&&s<0.02)||(s<0&&s>-0.02)){
            isDeadBand = true;
            return 0;
        }
        else{
            isDeadBand = false;
            return s;
        }
    }



    Vec_3d deathJudge3D(double n ,double m,double p_uni){
        Vec_3d temp;
        if(n*n+m*m>0.1*0.1||fabs(p_uni)>0.1){
            if(n*n+m*m>0.1*0.1){
                temp.x = n;
                temp.y = m;
            }
            else{
                temp.x = 0;
                temp.y = 0;
            }
            if(fabs(p_uni)>0.1){
                temp.w = p_uni;
            }
            else{
                temp.w = 0;
            }
            isDeadBand = false;
            return temp;
        }
        else{
            temp.x = 0;
            temp.y = 0;
            temp.w = 0;
            isDeadBand = true;
            return temp;
            
        }
    }


    bool GetDeadBand(){
        return isDeadBand;
    }

    
};



