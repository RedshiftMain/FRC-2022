package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.pcmConstants;

public class IntakeAndMag extends SubsystemBase {
    private final static WPI_TalonSRX intake = new WPI_TalonSRX(CANids.intakeId);
    //private final static TalonSRX magazine = new TalonSRX(CANids.magazineId);
    private final static WPI_TalonSRX lConveyor = new WPI_TalonSRX(CANids.lConveyorId);
    private final static WPI_TalonSRX rConveyor = new WPI_TalonSRX(CANids.rConveyorId);

    private final static DoubleSolenoid pistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pcmConstants.IntakeF, pcmConstants.IntakeR);

    public IntakeAndMag() {
        //Initialization
        intake.setInverted(false);
        //magazine.setInverted(false);
        lConveyor.setInverted(false);
        rConveyor.setInverted(true);

        intake.setNeutralMode(NeutralMode.Brake);
        //magazine.setNeutralMode(NeutralMode.Brake);
        lConveyor.setNeutralMode(NeutralMode.Coast);
        rConveyor.setNeutralMode(NeutralMode.Coast);

    }

    @Override
    public void periodic(){
        intake.feed();
        lConveyor.feed();
        rConveyor.feed();
    }

    public void activateIntake(boolean togg, double trigger) {
        //adss all methods into one, so its easier to call
        if (togg) {
            pistons.set(DoubleSolenoid.Value.kForward);
            intake.set(ControlMode.PercentOutput, 0.8);
        }
        else{
            pistons.set(DoubleSolenoid.Value.kReverse);
            intake.set(ControlMode.PercentOutput, 0);
        }
        funnelIn(trigger);
    }

    public void funnelIn(double bool) {
        if (bool!=0) {
            lConveyor.set(ControlMode.PercentOutput, 0.3);
            rConveyor.set(ControlMode.PercentOutput, 0.3);
            //intake.set(ControlMode.PercentOutput, 0.8);
            //magazine.set(ControlMode.PercentOutput, 1);
        } else {
            lConveyor.set(ControlMode.PercentOutput, 0);
            rConveyor.set(ControlMode.PercentOutput, 0);
            // 0   7 777777777777777777777777777777  7 7777777       7                                                                                                                                                                        7777ehintake.set(ControlMode.PercentOutput, 0);
            //magazine.set(ControlMode.PercentOutput, 0);j
        }

    }

    public static void quickRev(boolean test){
        if(test){
            lConveyor.set(ControlMode.PercentOutput, -1);
            rConveyor.set(ControlMode.PercentOutput, -1);
        }
        else{
            lConveyor.set(ControlMode.PercentOutput, 0);
            rConveyor.set(ControlMode.PercentOutput, 0);
        }
        
    }

    public static void Off() {
        pistons.set(DoubleSolenoid.Value.kOff);
    }

    
}
