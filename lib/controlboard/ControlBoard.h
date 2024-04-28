#include <frc/JoyStick.h>
#include "JoyStickPorts.h"
#include "../../frc2024/Ports.h"


enum JoyType
    {
        XBOX,
        PS4
    };
class ControlBoard
{
private:
    frc::Joystick main_joy{0};
    int joytype;
public:
    ControlBoard(int Type)
    {
        joytype = Type;
    }

    ~ControlBoard()
    {
    }

    double GetXSpeed()
    {
        if(joytype == XBOX){
            return -1.0 * main_joy.GetX();
        }
    }

    double GetYSpeed()
    {
        if(joytype == XBOX){
            return main_joy.GetY();
        }
        
    }

    double GetWSpeed()
    {
        if(joytype == XBOX){
            return -1.0 * main_joy.GetRawAxis(JoyStickPorts::Axis::RX);
        }
    }

    bool GetChassisResetToZero()
    {
        if(joytype == XBOX){
            return main_joy.GetRawButton(JoyStickPorts::Button::THREE_LINES);
        }
    }

    bool GetIsExMode()
    {
        if(joytype == XBOX){
            // return main_joy.GetRawButton(JoyStickPorts::Button::RB);
            return 0;
        }
    }
    
    double GetIsAutoAiming(){
        if(joytype == XBOX){
            return main_joy.GetRawAxis(JoyStickPorts::Axis::RA);
        }
    }
    double GetLeftTrigger(){
        if(joytype == XBOX){
            return main_joy.GetRawAxis(JoyStickPorts::Axis::LA);
        }
    }
    double GetRightTrigger(){
        if(joytype == XBOX){
            return main_joy.GetRawAxis(JoyStickPorts::Axis::RA);
        }
    }
    /**
     * @brief 获取住手柄POV按键，并将其整理为在区间+-180deg范围
    */
    double GetPOV_Pose(){
        double pov = main_joy.GetPOV();
        
        // default
        if(pov == -1){
            return -1;
        }
        else{
            if(pov > 180){
                pov -= 360;
            }
            return pov;
        }
    }
    /**
     * @brief 获取ABXY键位
    */
    int Get_ABXY(){
        if(main_joy.GetRawButton(JoyStickPorts::Button::A)){
            return JoyStickPorts::Button::A;
        }
        if(main_joy.GetRawButton(JoyStickPorts::Button::B)){
            return JoyStickPorts::Button::B;
        }
        if(main_joy.GetRawButton(JoyStickPorts::Button::X)){
            return JoyStickPorts::Button::X;
        }
        if(main_joy.GetRawButton(JoyStickPorts::Button::Y)){
            return JoyStickPorts::Button::Y;
        }
        if(main_joy.GetPOV() == 270){
            return 270;
        }
        if(main_joy.GetPOV() == 90){
            return 90;
        }
        else{
            return -1;
        }
    }
    
};
