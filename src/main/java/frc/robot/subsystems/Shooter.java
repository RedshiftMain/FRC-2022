package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.SpeedConstants;

public class Shooter extends SubsystemBase {
  private final static WPI_TalonFX lShooter = new WPI_TalonFX(CANids.lShooterId);
  private final static WPI_TalonFX rShooter = new WPI_TalonFX(CANids.rShooterId);
  //private final static AnalogInput ultrasonic = new AnalogInput(0);
  //private final static Ultrasonic ultrasonic2 = new Ultrasonic(0,0);
  // public DigitalOutput ultrasonicTriggerPin = new DigitalOutput(0); // Digital
  // Pin 0
  int x=0; //random int for placeholder

  public Shooter() {
    lShooter.configFactoryDefault();
    rShooter.configFactoryDefault();

    lShooter.configNeutralDeadband(0.1);
    rShooter.configNeutralDeadband(0.1);

    lShooter.setInverted(true);
    rShooter.setInverted(InvertType.OpposeMaster);
    // Left Shooter Max and Min configuration
    lShooter.configNominalOutputForward(1, 0);
    lShooter.configNominalOutputReverse(-1, 0);
    lShooter.configPeakOutputForward(1, 0);
    lShooter.configPeakOutputReverse(-1, 0);
    // Right Shooter Max and Min configuration
    rShooter.configNominalOutputForward(1, 0);
    rShooter.configNominalOutputReverse(-1, 0);
    rShooter.configPeakOutputForward(1, 0);
    rShooter.configPeakOutputReverse(-1, 0);

    // Sets motors to coast, not stop or continue
    lShooter.setNeutralMode(NeutralMode.Coast);
    rShooter.setNeutralMode(NeutralMode.Coast);
    // Sets time from neutral to full speed
    lShooter.configClosedloopRamp(0.5);
    rShooter.configClosedloopRamp(0.5);
    lShooter.configOpenloopRamp(0.5);
    rShooter.configOpenloopRamp(0.5);
    // Configures the Sensors built within the Talons
    lShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

  }

  @Override
  public void periodic() {
    lShooter.feed();
    rShooter.feed();

    SmartDashboard.putNumber("LSpeed", lShooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RSpeed", rShooter.getSelectedSensorVelocity());
  
    /*//Sensors are finnicky, so is my code
    if (x == 0) {
      Shooter.ultrasonicSensor();
    }
    x += 1;
    if (x == 30) {
      x = 0;
    }*/
  }

  /*public final static void ultrasonicSensor() {
    double rawValue = ultrasonic.getValue();
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    double currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;

    //double testInches = ultrasonic2.getRangeInches();

    if(currentDistanceInches > 108 || currentDistanceInches < 104){
      SmartDashboard.putNumber("Distance(In)", currentDistanceInches);
      //SmartDashboard.putNumber("Distance(In)v2", testInches);

    }
  }*/
  public static void quickRev(boolean bool){
    if(bool){
      rShooter.set(ControlMode.PercentOutput, -.1);
      lShooter.set(ControlMode.PercentOutput, -.1);
    }
    //else{
    //  rShooter.set(ControlMode.PercentOutput, 0);
    //  lShooter.set(ControlMode.PercentOutput, 0);
    //}
  }

  public final void shoot(double speed) {
    //Activates left motor, other motor follows so isn't necessary to code in
    rShooter.set(ControlMode.PercentOutput, speed * SpeedConstants.shootySpeed);
    lShooter.set(ControlMode.PercentOutput, speed * SpeedConstants.shootySpeed);
    SmartDashboard.putNumber("Shooter Speed", speed);
    
  }

  public void run() {
    lShooter.set(ControlMode.PercentOutput, .8);
  }

  public void stop() {
    lShooter.set(ControlMode.PercentOutput, 0);
  }

  public boolean atSpeed() {
    return lShooter.getSelectedSensorVelocity() >= 1.00;
  }
}
