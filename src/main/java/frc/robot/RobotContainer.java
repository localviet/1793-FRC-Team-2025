// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BargeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

//shuffle board type
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;//visualize data ig
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//limelight libraries
import frc.robot.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final BargeSubsystem m_BargeSubsystem = new BargeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Limelight m_limelight = new Limelight();  // Limelight subsystem
  //Limelight proportional constants, modify these to tune robot turning
  private final double KpHorizontal = 0.1;  // Proportional control constant for horizontal movement (tx)
  private final double KpVertical = 0.1;  // Proportional control constant for vertical movement (ty)

  private boolean fieldOriented = true;
  // The driver's controller
  private XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private XboxController m_grabberController = new XboxController(1);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                fieldOriented),
            m_robotDrive));

    m_driverController.getLeftY(); // Get Y-axis input for arm control

    // Bind the command to the Y-axis of the joystick
    m_ArmSubsystem.setDefaultCommand(
        new RunCommand(() -> m_ArmSubsystem.controlArm(m_grabberController.getLeftY()), m_ArmSubsystem)
    );
    //set commands

    autoChooser.setDefaultOption("Swerve Auto", swerveAutoCommand());
    autoChooser.addOption("Simple Command", getAutonomousCommand());
    autoChooser.addOption("Limelight Autonomous", limelightAutoCommand());  

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(() -> fieldOriented = !fieldOriented));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> m_elevator.setGoal(1.0).schedule()));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> m_elevator.setGoal(2.0).schedule()));

        new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> m_BargeSubsystem.toggleClamp()));

      
  }

  public XboxController getController(){
    return m_driverController;
  }

  public DriveSubsystem getDriveSysten(){
    return m_robotDrive;
  }

  public ElevatorSubsystem getElevatorSubsystem(){
    return m_elevator;
  }

  public BargeSubsystem getBargeSubsystem() {
    return m_BargeSubsystem;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command swerveAutoCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }


  //Limelight code IMPORTANT IF NO TARGET IS VISIBLE, STOPS ROBOT
  public Command limelightAutoCommand() {
    return new RunCommand(() -> {
      // Limelight data from the LimelightSubsystem
      double tx = m_limelight.getXOffset();  // Horizontal offset
      double ty = m_limelight.getYOffset();  // Vertical offset
      boolean targetVisible = m_limelight.hasValidTargets();  // Target visibility

      // If target is visible, adjust the robot's position
      if (targetVisible) {
        double steeringAdjust = KpHorizontal * tx;  // Steering correction based on horizontal offset (tx)
        double elevationAdjust = KpVertical * ty;  // Elevation correction based on vertical offset (ty)

          if (Math.abs(tx) < 1.0 && Math.abs(ty) < 1.0) {
           // If the robot is close to the tag, stop and complete an action
            m_robotDrive.drive(0, 0, 0, false);  // Stop the robot
            // coralShoot.activate();  // Incorporate shooting here
        } else {
          // Otherwise, keep moving toward the target
          m_robotDrive.drive(steeringAdjust, elevationAdjust, 0, false);  // Move towards target
        }

      } else {
        // If no target is visible, stop the robot
        m_robotDrive.drive(0, 0, 0, false);
      }
    }, m_robotDrive);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
