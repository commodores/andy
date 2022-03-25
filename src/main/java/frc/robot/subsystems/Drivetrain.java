// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C.Port;

public class Drivetrain extends SubsystemBase {

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final AHRS m_gyro;

  private final Field2d m_field;

  private final SwerveDriveOdometry m_odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveChannel, DriveConstants.kFrontLeftTurnChannel, DriveConstants.kFrontLeftEncoderChannel, false, true);
    m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveChannel, DriveConstants.kFrontRightTurnChannel, DriveConstants.kFrontRightEncoderChannel, false, true);
    m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveChannel, DriveConstants.kBackLeftTurnChannel, DriveConstants.kBackLeftEncoderChannel, false, true);
    m_backRight = new SwerveModule(DriveConstants.kBackRightDriveChannel, DriveConstants.kBackRightTurnChannel, DriveConstants.kBackRightEncoderChannel, false, true);

    m_gyro = new AHRS(SPI.Port.kMXP);
    
    m_field = new Field2d();

                                      
    m_odometry =
        new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(-m_gyro.getAngle()));

    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);
    
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void resetAll(){
    m_gyro.reset();
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
        m_field.setRobotPose(m_odometry.getPoseMeters());
    
        SmartDashboard.putNumber("Gyro", m_gyro.getYaw());
  }
  
}