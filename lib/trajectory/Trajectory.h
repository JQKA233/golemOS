#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include "choreo/lib/Choreo.h"
#include "choreo/lib/ChoreoSwerveCommand.h"
#include "choreo/lib/ChoreoTrajectory.h"
#include "choreo/lib/ChoreoTrajectoryState.h"

using namespace choreolib;
class Trajectory
{
private:
    Choreo chorpp;
    std::function<frc::Pose2d ()> Pose;
    ChoreoControllerFunction controler;
    std::function<void (frc::ChassisSpeeds)> outputchassisspeeds;
    std::function<bool ()> mirrortrajectory;
    ChoreoTrajectory traj = chorpp.GetTrajectory("Trajectory");
    ChoreoSwerveCommand choreoswervecommand
    {traj,Pose,controler,outputchassisspeeds,mirrortrajectory};

    
public:
    Trajectory(/* args */){

    }
    ~Trajectory(){

    }
};
// import com.choreo.lib.*;

// ChoreoTrajectory traj = Choreo.getTrajectory("Trajectory"); // 

// // 
// Choreo.choreoSwerveCommand(
//     traj, // 
//     this::getPose // 
//     new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
//     new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
//     new PIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // 
//     (ChassisSpeeds speeds) -> // 
//         this.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), ...),
//     () -> {
//         Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
//             mirror = alliance.isPresent() && alliance.get() == Alliance.Red;
//     }, // 
//     this, // 
// );
