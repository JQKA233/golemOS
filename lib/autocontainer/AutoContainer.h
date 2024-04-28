#pragma once

#include "team8011/frc2024/subsystems/AutoSub.h"
#include "team8011/lib/swerve/AutoChassis.h"
#include "frc/DriverStation.h"

class AutoContainer
{
private:

	std::vector<std::vector<double>> autoPath1{{1550,90},{1600,135},{1650,180}};

	std::vector<std::vector<double>> shootAction1{{0,30,-1},{1550,40,-1},{2300,30,-1}};

    AutoChassis autochassis;
    AutoSub autosub;
public:
    AutoContainer(){
    }
    ~AutoContainer(){
    }
    void Auto_Init(){
        // 自动底盘初始化
		autochassis.Chassis_Init_auto();
		autosub.AutoSubInit();
    }
    void Auto_Dis(){
        autosub.shootStop();
		autosub.Close_Path();
		autosub.Close_Intake();
    }
    void SixBallAuto_Red(){
        /**
		 * @brief 六球代码 比较稳
		*/
		autochassis.SetDrivePID_Slot(0);
		autosub.TurnPitchAngle(60);
		autosub.Open_Path(0.06);
		autosub.shootball_auto(14000);
		autosub.CanShootBall(8000);
		autosub.Open_Path(0.4);
		autosub.shootball_auto(11000);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-128,1880,30,1,1,0,1);
		autochassis.SetChassisTranslationAbs(65,1660,52,0,2,0,1);
		autosub.Open_Path(0.4);
				

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(180,690,30,1,1,0,1);
		autochassis.Turn_withAbsoluteModeNew(27,1,0);
		autochassis.SetChassisTranslationAbs(60,1100,49,0,2,0,0);
		autosub.Open_Path(0.4);
				
				
		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(152,850,34,1,1,0,1);
		autosub.Open_Path(0.4);
		// autochassis.Turn_withAbsoluteMode(0,0);
		// autochassis.Turn_withAbsFast(0,0);

		autochassis.setBallInOutAuto(0);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(178,5350,30,1,1,1,1,11000);
		autochassis.SetChassisTranslationAbs(-4,3600,25.2,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(18,0,1);
		autosub.Open_Path(0.4);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-155.5,3900,30,1,1,1,1,14000);
		autochassis.SetChassisTranslationAbs(24.5,3900,25.2,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(18,0,1);
		autochassis.setBallInOutAuto(0);
		autosub.Open_Path(0.4);

    }
    void SixBallAuto_Blue(){
        /**
		 * @brief 蓝色六球自动
		*/
		autochassis.SetDrivePID_Slot(0);
		autosub.TurnPitchAngle(60);
		autosub.Open_Path(0.06);
		autosub.shootball_auto(14000);
		autosub.CanShootBall(8000);
		autosub.Open_Path(0.4);
		autosub.shootball_auto(11000);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(128,1880,30,1,1,0,1);
		autochassis.SetChassisTranslationAbs(-65,1660,52,0,2,0,1);
		autosub.Open_Path(0.4);
				

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(180,690,30,1,1,0,1);
		autochassis.Turn_withAbsoluteModeNew(-27,1,0);
		autochassis.SetChassisTranslationAbs(-60,1100,49,0,2,0,0);
		autosub.Open_Path(0.4);
				
				
		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-152,850,34,1,1,0,1);
		autosub.Open_Path(0.4);
		// autochassis.Turn_withAbsoluteMode(0,0);
		// autochassis.Turn_withAbsFast(0,0);

		autochassis.setBallInOutAuto(0);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(-175,5350,30,1,1,1,1,11000);
		autochassis.SetChassisTranslationAbs(4,3600,25.2,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(-18,0,1);
		autosub.Open_Path(0.4);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(158,3820,30,1,1,1,1,14000);
		autochassis.SetChassisTranslationAbs(-22,3820,25.2,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(-18,0,1);
		autochassis.setBallInOutAuto(0);
		autosub.Open_Path(0.4);

    }
    void SixBallAuto_Red_Flip(){
        /**
         * @brief 红方六球，五球六球对调
        */
		autochassis.SetDrivePID_Slot(0);
		autosub.TurnPitchAngle(60);
		autosub.Open_Path(0.06);
		autosub.shootball_auto(14000);
		autosub.CanShootBall(8000);
		autosub.Open_Path(0.4);
		autosub.shootball_auto(11000);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-128,1880,30,1,1,0,1);
		autochassis.SetChassisTranslationAbs(65,1660,52,0,2,0,1);
		autosub.Open_Path(0.4);
				

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(180,690,30,1,1,0,1);
		autochassis.Turn_withAbsoluteModeNew(27,1,0);
		autochassis.SetChassisTranslationAbs(60,1100,49,0,2,0,0);
		autosub.Open_Path(0.4);
				
				
		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(152,850,34,1,1,0,1);
		autosub.Open_Path(0.4);
		// autochassis.Turn_withAbsoluteMode(0,0);
		// autochassis.Turn_withAbsFast(0,0);

		autochassis.setBallInOutAuto(0);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(-165,5350,30,1,1,1,1,11000);
		autochassis.SetChassisTranslationAbs(21,3600,25.2,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(18,0,1);
		autosub.Open_Path(0.4);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-169,3680,30,1,1,1,1,14000);
		autochassis.SetChassisTranslationAbs(-11,3680,25.2,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(18,0,1);
		autochassis.setBallInOutAuto(0);
		autosub.Open_Path(0.4);
    }
    void SixBallAuto_Blue_Flip(){
        /**
         * @brief 蓝方六球自动，五六球对调
        */
        autochassis.SetDrivePID_Slot(0);
		autosub.TurnPitchAngle(60);
		autosub.Open_Path(0.06);
		autosub.shootball_auto(14000);
		autosub.CanShootBall(8000);
		autosub.Open_Path(0.4);
		autosub.shootball_auto(11000);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(128,1880,30,1,1,0,1);
		autochassis.SetChassisTranslationAbs(-65,1660,52,0,2,0,1);
		autosub.Open_Path(0.4);
				

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(180,690,30,1,1,0,1);
		autochassis.Turn_withAbsoluteModeNew(-27,1,0);
		autochassis.SetChassisTranslationAbs(-60,1100,49,0,2,0,0);
		autosub.Open_Path(0.4);
				
				
		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-152,850,34,1,1,0,1);
		autosub.Open_Path(0.4);
		// autochassis.Turn_withAbsoluteMode(0,0);
		// autochassis.Turn_withAbsFast(0,0);

		autochassis.setBallInOutAuto(0);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(165,5350,30,1,1,1,1,11000);
		autochassis.SetChassisTranslationAbs(-21,3600,25.2,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(-18,0,1);
		autosub.Open_Path(0.4);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(169,3680,30,1,1,1,1,14000);
		autochassis.SetChassisTranslationAbs(11,3680,25.2,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(-18,0,1);
		autochassis.setBallInOutAuto(0);
		autosub.Open_Path(0.4);
    }
    void MidField_Red(){
        autochassis.SetDrivePID_Slot(1);
		autosub.TurnPitchAngle(60);
		autosub.Open_Path(0.06);
		autosub.shootball_auto(14000);
		autosub.CanShootBall(8000);
		autosub.Open_Path(0.4);
		autosub.shootball_auto(11000);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(150,5500,30,1,0,1,1,11000);
		autochassis.Turn_withAbsFast(50,0);
		// autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(135,2500,30,0,1,1,1);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(-37,5200,25,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(4,0,0);
		autosub.Open_Path(0.4);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(143,4000,30,1,0,1,1,14000);
		autochassis.Turn_withAbsFast(85,0);
		// autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(85,2500,30,0,1,1,1);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(-95,2500,30,0,2,1,1);
		autochassis.SetChassisTranslationAbs(-40,4000,28,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(5,0,0);
		autosub.Open_Path(0.4);
    }
    void MidField_Blue(){
        autochassis.SetDrivePID_Slot(1);
		autosub.TurnPitchAngle(60);
		autosub.Open_Path(0.06);
		autosub.shootball_auto(14000);
		autosub.CanShootBall(8000);
		autosub.Open_Path(0.4);
		autosub.shootball_auto(11000);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-150,5500,30,1,0,1,1,11000);
		autochassis.Turn_withAbsFast(-50,0);
		// autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(-135,2500,30,0,1,1,1);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(37,5200,25,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(-4,0,0);
		autosub.Open_Path(0.4);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-143,4000,30,1,0,1,1,14000);
		autochassis.Turn_withAbsFast(-85,0);
		// autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(-85,2500,30,0,1,1,1);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(95,2500,30,0,2,1,1);
		autochassis.SetChassisTranslationAbs(40,4000,28,0,2,1,1,14000);
		autochassis.Turn_withAbsoluteModeNew(-5,0,0);
		autosub.Open_Path(0.4);
    }

	void SixBallAuto_Red_New(){
        /**
		 * @brief 六球代码 比较稳
		*/
		autochassis.SetDrivePID_Slot(0);
		autosub.TurnPitchAngle(60);
		autosub.Open_Path(0.06);
		autosub.shootball_auto(15400);
		autosub.CanShootBall(8000);
		autosub.Open_Path(0.4);
		autosub.shootball_auto(12100);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-128,1800,30,1,1,0,1);
		autochassis.SetChassisTranslationAbs(65,1600,55,0,2,0,1);
		autosub.Open_Path(0.4);
				

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(180,950,30,1,1,0,1);
		autochassis.Turn_withAbsoluteModeNew(25.5,1,0);
		autochassis.SetChassisTranslationAbs(57,1140,46,0,2,0,0);
		autosub.Open_Path(0.4);
				
				
		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(151,1000,32.5,1,1,0,1);
		autosub.Open_Path(0.4);
		// autochassis.Turn_withAbsoluteMode(0,0);
		// autochassis.Turn_withAbsFast(0,0);

		autochassis.setBallInOutAuto(0);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(178,4950,30,1,1,2,1,12100);
		autochassis.SetChassisTranslationAbs(-4,2950,25,0,2,1,1,15000);
		autochassis.Turn_withAbsoluteModeNew(15.5,0,1);
		autosub.Open_Path(0.4);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-154.5,3530,30,1,1,1,1,15000);
		autochassis.SetChassisTranslationAbs(25.5,3350,24.7,0,2,1,1,15000);
		autochassis.Turn_withAbsoluteModeNew(15.5,0,1);
		autochassis.setBallInOutAuto(0);
		autosub.Open_Path(0.4);
    }

	void SixBallAuto_Red_New_Branch(){
        /**
		* @brief 六球代码 比较稳
		*/
		autochassis.SetDrivePID_Slot(0);
		autosub.TurnPitchAngle(60);
		autosub.Open_Path(0.06);
		autosub.shootball_auto(15400);
		autosub.CanShootBall(8000);
		autosub.Open_Path(0.4);
		autosub.shootball_auto(12100);

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(-128,1800,30,1,1,0,1);
		autochassis.SetChassisTranslationAbs(65,1560,55,0,2,0,1);
		autosub.Open_Path(0.4);
				

		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(180,950,30,1,1,0,1);
		autochassis.Turn_withAbsoluteModeNew(25.5,1,0);
		autochassis.SetChassisTranslationAbs(57,1140,44,0,2,0,0);
		autosub.Open_Path(0.4);
				
				
		autochassis.setBallInOutAuto(0);
		autochassis.SetChassisTranslationAbs(151,1000,31,1,1,0,1);
		autosub.Open_Path(0.4);
		// autochassis.Turn_withAbsoluteMode(0,0);
		// autochassis.Turn_withAbsFast(0,0);

		autochassis.setBallInOutAuto(0);
		autochassis.SetDrivePID_Slot(1);
		autochassis.SetChassisTranslationAbs(178,4950,30,1,1,2,1,12100,16000);
		autochassis.waitBallIn(0.7);
		if(autochassis.GetIsBallIn()){
			autochassis.SetChassisTranslationAbs(-4,2950,23,0,2,1,1,16000);
			autochassis.Turn_withAbsoluteModeNew(15.5,0,1,1.5);
			autosub.Open_Path(0.4);

			autochassis.setBallInOutAuto(0);
			autochassis.SetChassisTranslationAbs(-153.5,3580,30,1,1,1,1,16000);
			autochassis.SetChassisTranslationAbs(26.5,3250,23,0,2,1,1,16000);
			autochassis.Turn_withAbsoluteModeNew(16.5,0,1,1.5);
			autochassis.setBallInOutAuto(0);
			autosub.Open_Path(0.4);
		}
		else{
			autochassis.Turn_withAbsoluteModeNew(-90,0,0);
			autochassis.SetChassisTranslationAbs(-90,1600,30,0,1,0,1);
			autochassis.SetChassisTranslationAbs(25,3550,23,0,2,1,1,15000);
			autochassis.Turn_withAbsoluteModeNew(18,0,1,1.5);
			autochassis.setBallInOutAuto(0);
			autosub.Open_Path(0.4);
		}
    }

    void test(){
		autochassis.setInitAngle(60);
		autochassis.setBallInOutAuto(0);
		autochassis.Turn_withAbsoluteModeNew(0,0,0);
		autochassis.SetChassisTranslationAbs(180,1000,30,0,0,0,1);
		autochassis.SetChassisTranslationAbs(0,1000,30,0,0,0,1);
    }


};