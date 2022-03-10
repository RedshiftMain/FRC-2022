// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.Shooter;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static Shooter shooter = new Shooter();
    public static final class CANids{
        public static int compressorId = 0;
        public static int lMainFalconId = 4;
        public static int rMainFalconId = 1;
        public static int lSubFalconId = 3;
        public static int rSubFalconId = 2;
        public static int rShooterId = 11;
        public static int lShooterId = 12;
        public static int intakeId = 9;
        public static int magazineId = 6;
        public static int lConveyorId = 0;
        public static int rConveyorId = 8; 
        public static int lElevatorId = 17;
        public static int rElevatorId = 10;
    }

    public static final class pcmConstants{
        public static int IntakeF = 1;
        public static int IntakeR = 0;
        public static double pulseDur = 0.5;
    }

    public static final class Controls {

        public static int joystickPort = 0;
        //Front of controller
        public static int leftBumper = 5;
        public static int loadTrigger = 2;

        public static int rightBumper = 6;
        public static int shooterTrigger = 3;
        //Left side of controller
        public static int drivetrainLeft = 1;
        public static int xValJoyLeft = 0;
        //Right side of controller
        public static int drivetrainRight = 5;
        public static int xValJoyRight = 4;

        public static int backButton = 7;

        public static int xButton = 9;
        
    }

    public static final class ElevatorConstants {
        public static double positionTop = 30;
        public static double positionBottom = 0;
    }

    public static final class SpeedConstants {
        public static double driveSpeed = .85;
        public static double speedR = 0.5;
        public static double speedL = 0.5;
        public static double rampSpeed = 0.5;
        public static double distancePerPulse = (1.0 / 2048) * (.1524 * Math.PI) / 12.92;
        public static double elevatorSpeed = 0.2;
        public static double shootySpeed = 0.85;
        public static double motorSpeed = 0.75;

        
        
    }
    public static final class kGains {
        public static double desiredPercent = 0.50;//***** */
        public static double velocityPer100ms = 0.25;//***** */
        public static double kF = (desiredPercent * 1023) / velocityPer100ms;//***** 
        public static double kP = .25;
        public static double kI = 0.00025;
        public static double kD = 250;
        public static int kSlotIdx = 10;
        public static int kTimeoutMs = 10;
        public static int kPIDLoopIdx = 0;
        /*public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;*/
        public static final double kPDriveVel = 8.5;
        public static final double kTrackwidthMeters = 0.61;//******** 
        public static final DifferentialDriveKinematics kDDriveKinematics =/***** */
            new DifferentialDriveKinematics(kTrackwidthMeters);/***** */
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kRamseteB = 0.001;
        public static final double kRamseteZeta = 0.001;
        //public static final double kDrivekinematics = 0.7;
        public static final double ksPercent = 50;
        public static final double kvPercentSecondsPerMeter = 50;
        public static final double kaPercentSecondsSquaredPerMeter = 50;
    }

    public static final class ShooterCostants {
        //Constant for shooter
        public static int kTimeoutMs = 10;
        public static int kD = 10;
        public static int kPIDLoopIdx = 10;
    }

    public static final class Deadbands {
        public static double driveDeadband = 0.1;
        public static double elevatorDeadband = 0.1;
        public static double shooterDeadband = 0.1;
    }

    public static final class LimeLight {
        public static double drivetrainGearRatio = 155/12;
        public static double wheelBase = 73.538;
        public static double wheelCircumference = 18.85;
        public static double RPM = 4000.0;


    }

    public static final class Other {
        public static boolean compressorOn = true;
        public static boolean squaredSpeedInputs = true;

    }
}