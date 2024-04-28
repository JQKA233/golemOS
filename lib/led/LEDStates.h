#pragma once



class LEDStates
{
private:
    int mRed;
    int mGreen;
    int mBlue;
public:
    LEDStates(){
        mRed = 0;
        mGreen = 0;
        mBlue = 0;
    }
    LEDStates(int r,int g,int b){
        mRed = r;
        mGreen = g;
        mBlue = b;
    }
    int GetR(){
        return mRed;
    }
    int GetG(){
        return mGreen;
    }
    int GetB(){
        return mBlue;
    }
    ~LEDStates() {}
};
