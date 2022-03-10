package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer;
//import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.CANids;
import frc.robot.Constants.Deadbands;
//import frc.robot.Constants.SpeedConstants.*;
//import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private static WPI_TalonFX lElevator = new WPI_TalonFX(CANids.lElevatorId);
  private static WPI_TalonFX rElevator = new WPI_TalonFX(CANids.rElevatorId);

  public Elevator() {
    lElevator.configNeutralDeadband(Deadbands.elevatorDeadband);
    rElevator.follow(lElevator);
    lElevator.setInverted(false);
    rElevator.setInverted(true);
  }

  @Override
  public void periodic() {
    lElevator.feed();
    rElevator.feed();
    SmartDashboard.putNumber("Left Elevator", lElevator.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Elevator", rElevator.getSelectedSensorPosition());
  }

  //public void elevate(double speed) {
  //  lElevator.set(ControlMode.PercentOutput, SpeedConstants.elevatorSpeed*speed);
  //}

  //public double elevatorPos(){



//  return lElevator.getSelectedSensorPosition();
  //}
public void advElevator(double triggerL, double triggerR, boolean setVal){
  double targetPos;
  double targetPosR;
  boolean bool = false;
  
    if(setVal){
      bool = true;
    }
    if(!bool){
      lElevator.set(ControlMode.PercentOutput, triggerL * -0.5);
      rElevator.set(ControlMode.PercentOutput, triggerR * -0.5);
    }
    else{
      targetPos = lElevator.getSelectedSensorPosition();
      targetPosR = rElevator.getSelectedSensorPosition();
      lElevator.set(ControlMode.Position, targetPos);
      rElevator.set(ControlMode.Position, targetPosR);
    }
}

/*public void closeLooping( boolean act/*, int b, int c*///)//{

  //double motorOutput = lElevator.getMotorOutputPercent();

  //double targetPos;
  /*if(elevatorPos() > -0.1 && elevatorPos() < 0.5){
    targetPos = ElevatorConstants.positionTop;
  }
  else{
    targetPos = ElevatorConstants.positionBottom;
  }*/

  //if(act){
  //lElevator.set(ControlMode.Position, targetPos);
  //rElevator.set(ControlMode.Position, targetPos);

  }

 /* int maxGravityFF = b;
  int cosineScalar = c;

  lElevator.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, 
  maxGravityFF * cosineScalar);
  rElevator.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, 
  maxGravityFF * cosineScalar);
  */


