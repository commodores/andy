// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class RunTrajectory extends CommandBase {
  private Trajectory trajectory;

  DifferentialDriveVoltageConstraint autoVoltageConstraint = 
          /*new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(DriveConstants.kDriveEncoderResolution,
                                      DriveConstants.kDriveEncoderResolution,
                                      DriveConstants.kBackRightEncoderChannel),
          DriveConstants.kDriveKinematics,
          10);*/

  // Create config for trajectory
  // Create config for trajectory
  TrajectoryConfig config =
  new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
      .setReversed(true);
    
  TrajectoryConfig configBackwards =
      new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                          DriveConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          .setReversed(true)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint)
          .addConstraint(new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAccel));

      new thetaController(
        new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints
        ),
        thetaController.enableContinuousInput(-Math.PI, Math.PI)
      );
      private ProfiledPIDController thetaController;
      private Drivetrain m_swerve;
      private Drivetrain setModuleState;

// An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(
                //new Translation2d(1, 1), new Translation2d(2, -1)
                ),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(-3, 0, new Rotation2d(0)),
              config);

      

  SwerveControllerCommand forward = new SwerveControllerCommand(
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d()),
      new List.of(
        new Translation2d(3, 2),
        new Translation2d(1, 3)
      ),
      //new ArrayList<>(),
      new Pose2d(1, 1, new Rotation2d()),
      config
    ),Drivetrain::getPose,
    DriveConstants.kDriveKinematics,
    new PIDController(AutoConstants.kPXController, 0, 0),
    new PIDController(AutoConstants.kPYController, 0, 0),
    thetaController,
    m_swerve::setModuleStates,
    m_swerve
);

/** Creates a new AutoDrive. */
public RunTrajectory(String getPath) {
  // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.m_drivetrain);
  String path = getPath;
  if(path.equals("testPath")){
    trajectory = getTestPath();
  } else if(path.equals("driveOffTarmac")){
    trajectory = getDriveOffTarmacPath();
  } else {
    trajectory = getDriveOffTarmacPath();
  }  
}

@Override
  public void initialize() {
      ramsete.initialize();
  }

  @Override
  public void execute() {
      ramsete.execute();
  }

  @Override
  public void end(boolean interrupted) {
      ramsete.end(interrupted);
  }

  @Override
  public boolean isFinished() {
      return ramsete.isFinished();
  }

}