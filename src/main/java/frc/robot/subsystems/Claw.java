package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  public DoubleSolenoid clawPiston;
  public TalonFX clawMotor;

  public Claw() {
    if (!Constants.mantis) {
      clawPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
      clawMotor = new TalonFX(4);
      clawPiston.set(Value.kOff);
    }
  }

  public CommandBase motorOn() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, .2));
  }

  public CommandBase motorOff() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, 0));
  }

  public CommandBase close() {
    return run(() -> clawPiston.set(Value.kForward));
  }

  public CommandBase open() {
    return run(() -> clawPiston.set(Value.kReverse));
  }
}
