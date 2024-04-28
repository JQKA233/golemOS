#pragma once

#include "LEDStates.h"
#include "frc/AddressableLED.h"

enum{
    DISABLE,
    LOW_BATTERY,
    AIMED,
    READY_TO_SHOOT,
    LOSE_TARGET,
    AUTO_MODE,
};
class LEDControler
{
private:
    static constexpr int kLength = 72;
    // PWM port 9
    frc::AddressableLED *m_led;
    
    // Reuse the buffer
    std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

    // Store what the last hue of the first pixel is
    int firstPixelHue = 0;
    bool FlashState = 0;
    int TimePr_ms = 0;

public:
    LEDControler(int PWMPort){
        m_led = new frc::AddressableLED(PWMPort);
    }
    ~LEDControler(){
    }

    void LED_Init(){
        m_led->SetLength(kLength);
        m_led->SetData(m_ledBuffer);
        m_led->Start();
    }

    void LED_Dis(){
        // SetLEDs_Individually(0,0,0);
        Rainbow();
    }

    /**
     * @brief 设置颜色矩阵
    */
    void SetLEDs_inArray(LEDStates rgb_info){
        for (int i = 0; i < kLength; i++) {
            m_ledBuffer[i].SetRGB(rgb_info.GetR(), rgb_info.GetG(), rgb_info.GetB());
        }
        m_led->SetData(m_ledBuffer);
    }

    /**
     * @brief 设置RGB
    */
    void SetLEDs_Individually(int r,int g,int b){
        for (int i = 0; i < kLength; i++) {
            m_ledBuffer[i].SetRGB(r, g, b);
        }
        m_led->SetData(m_ledBuffer);
    }

    /**
     * @brief LED闪烁
     * @param T 频闪周期[3,+∞]以20ms为单位
     * @param Brightness 亮度[0,1]
     * @param color1 频闪颜色1
     * @param color2 频闪颜色2(默认为熄灯)
    */
    void FlashLight(int T,double Brightness,LEDStates color1,LEDStates color2 = LEDStates(0,0,0)){
        if(TimePr_ms == T*20){
            TimePr_ms = 0;
        }
        if(TimePr_ms < T*10){
            for (int i = 0; i < kLength; i++)
            {
                m_ledBuffer[i].SetRGB(Brightness * color1.GetR(),Brightness * color1.GetG(),Brightness * color1.GetB());
            }
        }
        if(TimePr_ms > T*10){
            for (int i = 0; i < kLength; i++)
            {
                m_ledBuffer[i].SetRGB(Brightness * color2.GetR(),Brightness * color2.GetG(),Brightness * color2.GetB());
            }
        }
        m_led->SetData(m_ledBuffer);
        TimePr_ms +=20;
    }

    /**
     * @brief 进度条显示
     * @param Percent 进度百分数[0,1]
     * @param Brightness 亮度[0,1]
     * @param color 进度条颜色
     * @param isCentralSymmetry 是否中心对称
    */
    void ProgressBar(int Percent,double Brightness,LEDStates color,bool isCentralSymmetry = 0,bool Direction = 0){
        if(isCentralSymmetry){
            if(Direction){
                for (int i = 0; i < kLength/2 * Percent; i++)
                {
                    m_ledBuffer[i].SetRGB(Brightness * color.GetR(),Brightness * color.GetG(),Brightness * color.GetB());
                }
                for (int i = kLength; i > kLength/2 * Percent ; i--)
                {
                    m_ledBuffer[i].SetRGB(Brightness * color.GetR(),Brightness * color.GetG(),Brightness * color.GetB());
                }
                m_led->SetData(m_ledBuffer);
            }
            else{
                for (int i = kLength/2 * (1-Percent); i < kLength/2 * (1+Percent); i++)
                {
                    m_ledBuffer[i].SetRGB(Brightness * color.GetR(),Brightness * color.GetG(),Brightness * color.GetB());
                }
                m_led->SetData(m_ledBuffer);
            }
            
        }
        else{
            if(Direction){
                for (int i = 0; i < kLength * Percent; i++)
                {
                    m_ledBuffer[i].SetRGB(Brightness * color.GetR(),Brightness * color.GetG(),Brightness * color.GetB());
                }
                m_led->SetData(m_ledBuffer);
            }
            else{
                for (int i = kLength * (1-Percent); i < kLength; i++)
                {
                    m_ledBuffer[i].SetRGB(Brightness * color.GetR(),Brightness * color.GetG(),Brightness * color.GetB());
                }
                m_led->SetData(m_ledBuffer);
            }
            
        }
        
    }

    /**
     * @brief 机器人状态设置LED
     * @param state 机器人状态
     * @param Brightness 亮度[0,1]
    */
    void SetLED_byStates(int state,double Brightness = 1){
        // 电量低 红色频闪
        if(state == LOW_BATTERY){
            FlashLight(20,1,LEDStates(255,0,0),LEDStates(0,0,0));
        }
        if(state == AIMED){
            // 已瞄准 黄色常亮
            SetLEDs_Individually(Brightness * 255,Brightness * 60,Brightness * 0);
        }
        if(state == READY_TO_SHOOT){
            // 可以射球 绿色常亮
            SetLEDs_Individually(Brightness * 0,Brightness * 255,Brightness * 0);
        }
        if(state == LOSE_TARGET){
            // 丢失目标 红色常亮
            SetLEDs_Individually(Brightness * 255,Brightness * 0,Brightness * 0);
        }
        if(state == AUTO_MODE){
            // 自动阶段 绿色常亮
            SetLEDs_Individually(Brightness * 0,Brightness * 255,Brightness * 50);
        }
        if(state == DISABLE){
            SetLEDs_Individually(0,0,0);
        }
    }

    /**
     * @brief 彩虹模式，系统自带
    */
    void Rainbow(){
        // For every pixel
        for (int i = 0; i < kLength; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
            // Set the value
            m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        firstPixelHue += 3;
        // Check bounds
        firstPixelHue %= 180;

        // Set the LEDs
        m_led->SetData(m_ledBuffer);
    }
};
