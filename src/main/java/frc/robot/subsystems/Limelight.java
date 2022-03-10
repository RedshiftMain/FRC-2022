package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

/*
 * TODO: turns shooter wheel, magazine wheel, and intake line on AFTER YOU AIM
 */
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    public static Drivetrain drivetrain;
    public static Shooter shooter;
    public IntakeAndMag intakeAndMag;

    public static double driveLeft = 0.5; // Will be a constant, testing needs to be done
    public static double driveRight = 0.5; // Will be a constant, testing needs to be done
    public double shooterNum = 0.8; // Percentage of power the shooter will run at
    public static double stopMotors = 0.0; // used to set motors to stop

    public static double thetaToTarget; // Angle, positive is right, negative is left
    public static double distanceToTarget; // Horizontal in feet
    public double targetPlaneSize; // Vertical height in feet
    public static double time; // In minutes

    public boolean booleanIaM; // Boolean for Intake and Magazine
    public boolean ran = false; // Boolean to determine if the code has run
    public double loadVal; // Trigger value for the magazine
    public static boolean inversion = false; // Determines if the robot turns right(false) or left(true)
    public static boolean isAimed = false; // Determines if the robot has aimed itself

    static double ballVelocity;
    static double timeInAir;
    static double powerNeeded;
    static double ballMass = 0.249576; // kg
    static double powerPercentage;

    double height;
    double planeHeight;
    // static boolean fired = false;

    public Limelight(Drivetrain d, Shooter s, IntakeAndMag iam) {
        // Use addRequirements() here to declare subsystem dependencies.
        drivetrain = d;
        shooter = s;
        intakeAndMag = iam;
    }

    // turns the limelight subsystem into a command
    public void subToCommand(boolean run) {

        if (run) {
            limeInitialize();
            while (!limeIsFinished()) {
                limeExecute();
            }
            limeEnd(false);
        }
    }

    // Called when the command is initially scheduled.

    public void limeInitialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
    }

    // Called every time the scheduler runs while the command is scheduled.

    public void limeExecute() {
        // gets numerical (floating point) 'tv' value from the limelight
        // network table
        // if limelight 'tv' label == 0, then it doesn't have target
        // 1 = has target
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) == 0) {
            drivetrain.drive(driveLeft * -1,
                    driveRight, inversion);
            // runs for a time determined by math
            Timer.delay(60);
            // stops motors
            drivetrain.drive(stopMotors,
                    stopMotors, inversion);
        }
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) == 1) {
            // thetaToTarget = cameraInfo[4];
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

            drivetrain.drive(driveLeft * -1,
                    driveRight * -1, inversion);
            // runs for a time determined by math
            Timer.delay(time * 60);
            // stops motors
            drivetrain.drive(stopMotors,
                    stopMotors, inversion);
        }

    }

    public void limeEnd(boolean interrupted) {
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

            planeHeight = (120
                    / NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0.0)) * 2;
            height = (planeHeight / 2)
                    * NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty0").getDouble(0.0);
            distance = (Math.cos(limelightAngle) * height) / (Math.cos(90 - limelightAngle));
            while (!atDist) {
                if (distance < 100) {// aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
                    drivetrain.drive(driveLeft * -1,
                            driveRight * -1, false);
                } else if (distance > 140) {// aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
                    drivetrain.drive(driveLeft,
                            driveRight, false);
                } else {
                    atDist = true;
                    drivetrain.drive(stopMotors,
                            stopMotors, inversion);
                }
            }
            shooter.shoot(.8);// aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
            intakeAndMag.activateIntake(false, 1.00);
            Timer.delay(120);
            shooter.shoot(0);
            intakeAndMag.activateIntake(false, 0);
        }
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

    }

    public boolean limeIsFinished() {
        if (thetaToTarget > -0.5 && thetaToTarget < 0.5) {
            return true;
        } else {
            return false;
        }

    }
}
