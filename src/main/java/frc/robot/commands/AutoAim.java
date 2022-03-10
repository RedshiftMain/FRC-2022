// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeAndMag;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.*;

public class AutoAim extends CommandBase {
    /** Creates a new AutoAim. */
    public static Drivetrain drivetrain;
    public static Shooter shooter;
    public IntakeAndMag intakeAndMag;

    public static double driveLeft = 0.5; // Will be a constant, testing needs to be done
    public static double driveRight = 0.5; // Will be a constant, testing needs to be done
    public double shooterNum = 0.8; // Percentage of power the shooter will run at
    public static double stopMotors = 0.0; // used to set motors to stop

    public static double thetaToTarget = 0.0; // Angle, positive is right, negative is left
    public static double distanceToTarget; // Horizontal in feet
    public double targetPlaneSize; // Vertical height in feet
    public static double time = 0.0; // In minutes

    public boolean booleanIaM; // Boolean for Intake and Magazine
    public boolean ran = false; // Boolean to determine if the code has run
    public double loadVal; // Trigger value for the magazine
    public static boolean inversion = false; // Determines if the robot turns right(false) or left(true)
    public static boolean isAimed = false; // Determines if the robot has aimed itself
    // public static double[] arrBlank = new double[6];

    // static NetworkTable table =
    // NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
    // static NetworkTableEntry camTran = table.getEntry("camtran");

    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);
    // static double[] cameraInfo = camTran.getDoubleArray(arrBlank);

    static double ballVelocity;
    static double timeInAir;
    static double powerNeeded;
    static double ballMass = 0.249576; // kg
    static double powerPercentage;

    double height;
    double planeHeight;
    // static boolean fired = false;
    public Timer timer = new Timer();

    public AutoAim(Drivetrain d, Shooter s, IntakeAndMag iam) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(d);
        addRequirements(s);
        addRequirements(iam);
        drivetrain = d;
        shooter = s;
        intakeAndMag = iam;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) == 0) {
            drivetrain.drive(driveLeft * -1, driveRight, inversion);
            // runs for a time determined by math
            // Timer.delay( 60 ); Big problem
            // stops motors
            if (timer.advanceIfElapsed(0.1)) {
                drivetrain.drive(stopMotors,
                        stopMotors, inversion);
            }
        } else {
            // thetaToTarget = 29;
            thetaToTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
            // math for time to turn in minutes
            time = (LimeLight.RPM) / ((thetaToTarget * (LimeLight.wheelBase / 2)) / LimeLight.wheelCircumference);
            // left or right
            if (thetaToTarget < 0.4) {
                inversion = true;
            } else if (thetaToTarget > -0.4) {
                inversion = false;
            } else {
                isAimed = true;
            }

            drivetrain.drive(driveLeft,
                    driveRight * -1, inversion);
            // runs for a time determined by math
            // Timer.delay( time * 60 );
            // stops motors
            if (timer.advanceIfElapsed(time / 60)) {
                drivetrain.drive(stopMotors,
                        stopMotors, inversion);
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        double limelightAngle = 7;
        double distance;
        boolean atDist = false;
        // determines the percentage of power for the shooter motors to fire
        if (!interrupted) {
            // camTran =
            // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran");
            // distanceToTarget = cameraInfo[0];
            /*
             * ballVelocity = Math.sqrt( (distanceToTarget*9.8) / (Math.sin(2*70)) );
             * powerNeeded = (ballMass * ballVelocity * distanceToTarget )/ (timeInAir *
             * timeInAir);
             * //need to transfer power to percentage of motor
             * powerPercentage = ( powerNeeded / 3 ) / 100; //power over radius of wheel
             * 
             * shooter.shoot(powerPercentage);
             * intakeAndMag.activateIntake(false, 1.00);
             */

            /*
             * planeHeight = (120 /
             * NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").
             * getDouble(0.0) ) * 2;
             * height = (planeHeight/2) *
             * NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty0").
             * getDouble(0.0);
             * distance = (Math.cos(limelightAngle) * height) /
             * (Math.cos(90-limelightAngle));
             * while(!atDist){//Shouldnt be a while loop
             * TODO: Fix this
             * if(distance < 100){//aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
             * drivetrain.drive(driveLeft * -1,
             * driveRight * -1, false);
             * }
             * else if(distance > 140){//aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
             * drivetrain.drive(driveLeft ,
             * driveRight , false);
             * }
             * else{
             * atDist = true;
             * drivetrain.drive(stopMotors,
             * stopMotors, inversion);
             * }
             */
            // }
            // shooter.shoot(.8);//aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
            // intakeAndMag.activateIntake(false, 1.00);
            // Timer.delay(120);
            // if(timer.advanceIfElapsed(5)){
            // shooter.shoot(0);
            // intakeAndMag.activateIntake(false, 0);}
        }
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (thetaToTarget > -0.5 && thetaToTarget < 0.5) {
            return true;
        } else {
            return false;
        }
    }
}
