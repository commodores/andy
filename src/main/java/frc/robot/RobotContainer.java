// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.DriveManual;
import frc.robot.commands.ResetAll;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static final XboxController m_controller = new XboxController(0);
  public static final XboxController m_controller2 = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  public static Intake m_intake = new Intake();
  public static Arm m_arm = new Arm();

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    /* Initialize various systems on robotInit. */
    this.initializeStartup();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_controller2, Button.kA.value)
        .whileHeld(() -> m_intake.runIntake(-.8))
        .whenReleased(() -> m_intake.runIntake(0));

    new JoystickButton(m_controller2, Button.kB.value)
        .whileHeld(() -> m_intake.runIntake(.8))
        .whenReleased(() -> m_intake.runIntake(0));

    new JoystickButton(m_controller2, Button.kX.value)
        .whileHeld(() -> m_arm.moveArm(-.4))
        .whenReleased(() -> m_arm.moveArm(0));

    new JoystickButton(m_controller2, Button.kY.value)
        .whileHeld(() -> m_arm.moveArm(.2))
        .whenReleased(() -> m_arm.moveArm(0));

  }

  private void initializeStartup()
  {
    m_swerve.setDefaultCommand(
      new DriveManual(m_swerve,true)); //Setup default command for drivetrain and set field relative or robot relative drive

    SmartDashboard.putData("Reset All", new ResetAll(m_swerve));

  }
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    m_intake.runIntake(.5);
    
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(true);

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

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_swerve::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_swerve::setModuleStates,
            m_swerve);

    // Reset odometry to the starting pose of the trajectory.
    m_swerve.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_swerve.drive(0, 0, 0, false)).andThen(() -> m_intake.runIntake(0));
  }
}
  /**
   * Set options for autonomous command chooser and display them for selection on the SmartDashboard.
   * Using string chooser rather than command chooser because if using a command chooser, will instantiate
   * all the autonomous commands. This may cause problems (e.g. initial trajectory position is from a
   * different command's path).
   */
  private void initializeAutoChooser()
  {
    /* Add options (which autonomous commands can be selected) to chooser. */
    m_autoChooser.setDefaultOption("Do Nothing", "doNothing");
    m_autoChooser.addOption("Test It", "testPath");
    m_autoChooser.addOption("Drive off Tarmac", "offTarmac");
    m_autoChooser.addOption("1 ball", "ball1");
    m_autoChooser.addOption("2 ball", "ball2");
    m_autoChooser.addOption("3 ball", "ball3");
    m_autoChooser.addOption("4 ball", "ball4");
    m_autoChooser.addOption("5 ball", "ball5");
    m_autoChooser.addOption("Mid RedBall Defense", "doubleRedDefense");

    /* Display chooser on SmartDashboard for operators to select which autonomous command to run during the auto period. */
    SmartDashboard.putData("Autonomous Command", m_autoChooser);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    switch (m_autoChooser.getSelected())
    {
      case "testPath" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new RunTrajectory("testPath");

      case "offTarmac":
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new FlashyMove();

      case "ball1" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new AppleSauceNumberOne();

      case "ball2" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new TwoFish();

      case "ball3" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new RedFish();

      case "ball4" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new BlueFish();

      case "ball5" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new LeeroyJenkins();

      case "doubleRedDefense" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new DDayDefense();

      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: " + m_autoChooser.getSelected() + "\n");
        return null;
    }
  }
}