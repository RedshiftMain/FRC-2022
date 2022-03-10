package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.Deadbands;

import static frc.robot.Constants.CANids.*;
import static frc.robot.Constants.SpeedConstants.*;
import static frc.robot.Constants.kGains.*;

public class Drivetrain extends SubsystemBase {
  private final WPI_TalonFX lMainFalcon = new WPI_TalonFX(lMainFalconId);
  private final WPI_TalonFX rMainFalcon = new WPI_TalonFX(rMainFalconId);
  private final WPI_TalonFX lSubFalcon = new WPI_TalonFX(lSubFalconId);
  private final WPI_TalonFX rSubFalcon = new WPI_TalonFX(rSubFalconId);
  private final DifferentialDrive drive = new DifferentialDrive(rMainFalcon, lMainFalcon);

  private final static AHRS gyro = new AHRS(Port.kMXP);
  private final static edu.wpi.first.math.kinematics.DifferentialDriveOdometry odometry = 
    new DifferentialDriveOdometry(getHeading());
  //private int inverter = 1;

  public Drivetrain() {
    lMainFalcon.configFactoryDefault();
    rMainFalcon.configFactoryDefault();
    lSubFalcon.configFactoryDefault();
    rSubFalcon.configFactoryDefault();

    // lSubFalcon.set(ControlMode.Follower, lMainFalcon.getDeviceID());
    // rSubFalcon.set(ControlMode.Follower, rMainFalcon.getDeviceID());

    lSubFalcon.follow(lMainFalcon);
    rSubFalcon.follow(rMainFalcon);

    lMainFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, 0);
    rMainFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, 0);

    lMainFalcon.setInverted(true);
    rMainFalcon.setInverted(true);
    lSubFalcon.setInverted(InvertType.FollowMaster);
    rSubFalcon.setInverted(InvertType.FollowMaster);

    lMainFalcon.setNeutralMode(NeutralMode.Brake);
    rMainFalcon.setNeutralMode(NeutralMode.Brake);
    lSubFalcon.setNeutralMode(NeutralMode.Brake);
    rSubFalcon.setNeutralMode(NeutralMode.Brake);

    lMainFalcon.configOpenloopRamp(rampSpeed);
    rMainFalcon.configOpenloopRamp(rampSpeed);

    drive.setDeadband(Deadbands.driveDeadband);
    lMainFalcon.configNeutralDeadband(0.04, kTimeoutMs);
    lMainFalcon.configNeutralDeadband(0.04, kTimeoutMs);

    reset();
  }

  @Override
  public void periodic() {
    lMainFalcon.feed();
    rMainFalcon.feed();
    lSubFalcon.feed();
    rSubFalcon.feed();
    odometry.update(getHeading(), lMainFalcon.getSelectedSensorPosition()*distancePerPulse, 
    rMainFalcon.getSelectedSensorPosition()*distancePerPulse);
    SmartDashboard.putNumber("Left Encoder", lMainFalcon.getSelectedSensorPosition());                  
    SmartDashboard.putNumber("Right Encoder", rMainFalcon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left sub Encoder", lSubFalcon.getSelectedSensorPosition());                  
    SmartDashboard.putNumber("Righ sub Encoder", rSubFalcon.getSelectedSensorPosition());
  }

  public double LdriveGetPercent(){
    return lMainFalcon.getSelectedSensorVelocity();
  }
  public double RdriveGetPercent(){
    return rMainFalcon.getSelectedSensorVelocity();
  }
  public static void driveGetVel(double v1, double v2){
    
  }

  public void drive(double rSpeed, double lSpeed, boolean inversion) {
    //if(inversion){
    //  inverter = -1;
    //}
    drive.tankDrive(rSpeed*driveSpeed/*inverter*/, lSpeed*driveSpeed*-1/*inverter*/, true);
    //inverter = 1;
  }

  private static Rotation2d getHeading() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360));
  }

  public static Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(lMainFalcon.getSelectedSensorVelocity(), 
    lMainFalcon.getSelectedSensorVelocity());
    //may need distance per pulse multiplication
  }

  public double rightEncoder() {
    return rMainFalcon.getSelectedSensorPosition()*distancePerPulse;
  }

  public double leftEncoder() {
    return lMainFalcon.getSelectedSensorPosition()*distancePerPulse;
  }

  public void reset() {
    lMainFalcon.setSelectedSensorPosition(0);
    rMainFalcon.setSelectedSensorPosition(0);
    odometry.resetPosition(odometry.getPoseMeters(), getHeading());
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(pose, getHeading());
  }

  public void autonStuff(){
    lMainFalcon.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		lMainFalcon.config_kF(kSlotIdx, kF, kTimeoutMs);
		lMainFalcon.config_kP(kSlotIdx, kP, kTimeoutMs);
		lMainFalcon.config_kI(kSlotIdx, kI, kTimeoutMs);
		lMainFalcon.config_kD(kSlotIdx, kD, kTimeoutMs);
    lMainFalcon.configNeutralDeadband(0.001, kTimeoutMs);

    rMainFalcon.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		rMainFalcon.config_kF(kSlotIdx, kF, kTimeoutMs);
		rMainFalcon.config_kP(kSlotIdx, kP, kTimeoutMs);
		rMainFalcon.config_kI(kSlotIdx, kI, kTimeoutMs);
		rMainFalcon.config_kD(kSlotIdx, kD, kTimeoutMs);
    rMainFalcon.configNeutralDeadband(0.001, kTimeoutMs);

    lMainFalcon.setSensorPhase(true);
    rMainFalcon.setSensorPhase(true);
  }

  public void autonDTreset(){
    lMainFalcon.configNeutralDeadband(0.04, kTimeoutMs);
    lMainFalcon.configNeutralDeadband(0.04, kTimeoutMs);
  }

  
}