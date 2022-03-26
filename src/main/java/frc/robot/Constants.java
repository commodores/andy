// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        
        //Robot Characteristics
        public static final double kWheelDiameter = 0.0762;

        public static final double kTrackWidth = 0.6096;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.6096;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kDriveEncoderResolution = 2048;
        public static final double kDriveGearRatio = 5.25;

        //Power Management for Drivetrain
        public static StatorCurrentLimitConfiguration kTalonCurrentConfig = new StatorCurrentLimitConfiguration(true, 80, 60, 1.0);

        public static int kVoltageCompensation = 12;

        public static int kRevContinuosCurrentLimit = 20;
        public static int kRevPeakCurrentLimit = 30;
        
        //CAN IDS for Motors and Encoders
        public static final int kFrontLeftDriveChannel = 10;
        public static final int kFrontLeftTurnChannel = 11;
        public static final int kFrontLeftEncoderChannel = 12;

        public static final int kFrontRightDriveChannel = 13;
        public static final int kFrontRightTurnChannel = 14;
        public static final int kFrontRightEncoderChannel = 15;

        public static final int kBackLeftDriveChannel = 16;
        public static final int kBackLeftTurnChannel = 17;
        public static final int kBackLeftEncoderChannel = 18;

        public static final int kBackRightDriveChannel = 19;
        public static final int kBackRightTurnChannel = 20;
        public static final int kBackRightEncoderChannel = 21;

        public static final double kMaxSpeedMetersPerSecond = 2.84;

        public static final double kMaxAccelerationMetersPerSecondSquared = 0;

        public static final double kMaxCentripetalAccel = 1.5;

        //arm CAN ID is 6; Intake/Shooter ID is 7
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * (2 * Math.PI);
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * (2 * Math.PI);
    
        public static final double kPModuleTurningController = 4;//3.5
    
        public static final double kPModuleDriveController = 1.82;

        public static final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.578,2.36);
        public static final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(.952,.753);
      }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 30;
    }

    public static final class ArmConstants {
        public static final int karmMotorPort = 31;
    }

//Drive Characterization
//kS - 0.578
//kV - 2.36
//kA - .123
//kP - 1.82

//Turn Characterization
//kS - 0.952
//kV - 0.753
//kA - 0.0371
//kP - 1.24
}