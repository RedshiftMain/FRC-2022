// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Controls.*;

//import frc.robot.subsystems.Camera;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.ScheduleCommand;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Controls;
//import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.kGains;
import frc.robot.commands.AutoAim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeAndMag;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
//import edu.wpi.first.wpilibj.AnalogInput;
//import frc.robot.commands.*;
//import frc.robot.commands.AutoAim;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer{
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooter = new Shooter();
  private final IntakeAndMag intakeAndMag = new IntakeAndMag();
  private final Elevator elevator = new Elevator();
  public Joystick controller = new Joystick(joystickPort);
  public Joystick eleControl = new Joystick(1);
  //public Limelight limelight = new Limelight(drivetrain, shooter, intakeAndMag);
  
  
  
  TrajectoryConfig config =
    new TrajectoryConfig(
        kGains.kMaxSpeedMetersPerSecond,
        kGains.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(kGains.kDDriveKinematics);

  Trajectory straightTrajectory =
    TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, 1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config);

  Trajectory turnaroundTrajectory =
    TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, 1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(Math.PI)),
      // Pass config
      config);
    //public Camera cam = new Camera();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @param List 
   * 
   * @p aram shooterGo
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //Camera.videoToDashboard();

    
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

 
  private void configureButtonBindings() {
    
    shooter.setDefaultCommand(
      new RunCommand(() -> shooter.shoot(controller.getRawAxis(shooterTrigger)*-1), shooter));
    drivetrain.setDefaultCommand(
      new RunCommand(() -> drivetrain.drive(controller.getRawAxis(drivetrainLeft) * -1,
      controller.getRawAxis(drivetrainRight) * -1, false), drivetrain));
    intakeAndMag.setDefaultCommand(
      new RunCommand(() -> intakeAndMag.activateIntake(controller.getRawButton(Controls.leftBumper), 
      controller.getRawAxis(Controls.loadTrigger)), intakeAndMag));
    elevator.setDefaultCommand(new RunCommand(() -> 
      elevator.advElevator(eleControl.getRawAxis(drivetrainRight), eleControl.getRawAxis(drivetrainLeft), eleControl.getRawButton(rightBumper)), elevator));
    //new JoystickButton(controller, Controls.xButton).whenPressed(new RunCommand(() -> 
    //  IntakeAndMag.quickRev(controller.getRawButton(xButton))).andThen(new RunCommand(() -> 
    //  Shooter.quickRev(controller.getRawButton(xButton)))));
    new JoystickButton(controller, rightBumper).toggleWhenPressed(new AutoAim(drivetrain, shooter, intakeAndMag));
  }
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    RamseteCommand ramseteCommand =
      new RamseteCommand(
        straightTrajectory,
        Drivetrain::getPose,
        new RamseteController(kGains.kRamseteB, kGains.kRamseteZeta),
          
        kGains.kDDriveKinematics,
        
        Drivetrain::driveGetVel,
        
        drivetrain);

        drivetrain.resetOdometry(straightTrajectory.getInitialPose());

    RamseteCommand ramseteCommand2 =
      new RamseteCommand(
        turnaroundTrajectory,
        Drivetrain::getPose,
        new RamseteController(kGains.kRamseteB, kGains.kRamseteZeta),
            
        kGains.kDDriveKinematics,

        Drivetrain::driveGetVel,
        
        drivetrain);

        drivetrain.resetOdometry(turnaroundTrajectory.getInitialPose());
        

    // An ExampleCommand will run in autonomous
    /*return ramseteCommand.andThen(() -> drivetrain.drive(0.5,0.5, false), drivetrain ).
      andThen(() -> intakeAndMag.activateIntake(true, 0.0), intakeAndMag).andThen(ramseteCommand2).
      andThen(() -> drivetrain.drive(0.0,0.0, false), drivetrain ).
      andThen(() -> intakeAndMag.activateIntake(false, 0.0), intakeAndMag).
      andThen(new AutoAim(drivetrain, shooter, intakeAndMag) );*/
    //Make a command in autonCode
    //return ramseteCommand;
    return ramseteCommand.andThen(new RunCommand(() -> intakeAndMag.activateIntake
    (controller.getRawButton(Controls.leftBumper), controller.getRawAxis(Controls.loadTrigger)), intakeAndMag)).
    andThen(ramseteCommand2).andThen(new AutoAim(drivetrain, shooter, intakeAndMag));
  }
}
