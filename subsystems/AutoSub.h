#pragma once

#include "team8011/lib/drivers/LazyTalonFX.h"
#include "frc/Timer.h"

class AutoSub
{
private:
    LazyTalonFX shoot1_auto{10};
    LazyTalonFX shoot2_auto{11};
    // LazyTalonFX Pitch_auto{21};
    TalonFX Pitch_auto = TalonFX(21);
    LazyTalonFX Intake_auto{22};
    // LazyTalonFX shootPath_auto{16};
    TalonFX shootPath_auto = TalonFX(16);

    double centerAngUnit_auto = 1613;
    double Angdeg_auto = 30;

public:

    AutoSub(){
        centerAngUnit_auto = 1613;
    }
    ~AutoSub(){

    }

    void AutoSubInit()
    {
        Close_Intake();
        Close_Path();
        shootStop();
    }
    void Delay_s(double s){
        frc::Wait((units::time::second_t)s);
    }
    void Open_Intake()
    {
        Intake_auto.set(ControlMode::PercentOutput,1);
    }

    void Close_Intake()
    {
        Intake_auto.set(ControlMode::Disabled,0);
    }

    void Open_Path(double Output)
    {
        // shootPath_auto.set(ControlMode::PercentOutput,Output);
        // shootPath_auto.set(ControlMode::PercentOutput,Output);
        shootPath_auto.Set(ControlMode::PercentOutput,Output);
        shootPath_auto.Set(ControlMode::PercentOutput,Output);
    }

    void Close_Path()
    {
        // shootPath_auto.set(ControlMode::Disabled,0);
        shootPath_auto.Set(ControlMode::Disabled,0);
    }

    void TurnPitchAngle(double Angle_auto)
    {
        // Pitch_auto.set(ControlMode::Position,centerAngUnit_auto+Angle_auto/360.0*4096);
        Pitch_auto.Set(ControlMode::Position,centerAngUnit_auto+Angle_auto/360.0*4096);
    }

    void shootball_auto(double Speed)
    {
        shoot1_auto.set(ControlMode::Velocity,Speed);
        shoot2_auto.set(ControlMode::Velocity,Speed);
    }

    void shootStop()
    {
        shoot1_auto.set(ControlMode::Disabled,0);
        shoot2_auto.set(ControlMode::Disabled,0);
    }

    void shootPercent(double percOut){
        shoot1_auto.set(ControlMode::PercentOutput,percOut);
        shoot2_auto.set(ControlMode::PercentOutput,percOut);
    }

    void shootStopFast()
    {
        shoot1_auto.set(ControlMode::PercentOutput,0);
        shoot2_auto.set(ControlMode::PercentOutput,0);
    }

    void CanShootBall(double speedRota)
    {
        while(speedRota-shoot1_auto.superTalonFX->GetSelectedSensorVelocity()>300){}
    }

};