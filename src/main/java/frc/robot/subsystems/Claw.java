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

  public DoubleSolenoid clawPiston1;
  public DoubleSolenoid clawPiston2;
  public TalonFX clawMotor;

  public Claw() {
    if (!Constants.mantis) {
      clawPiston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
      clawPiston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4); //idk
      clawMotor = new TalonFX(4);
      clawPiston1.set(Value.kOff);
      clawPiston2.set(Value.kOff);
    }
  }

  public CommandBase motorOn() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, .2));
  }

  public CommandBase motorOff() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, 0));
  }

  public CommandBase closeCube() {
    return run(() -> clawPiston1.set(Value.kForward))
      .andThen(run(() -> clawPiston2.set(Value.kReverse)));
  }

  public CommandBase openCube() {
    return run(() -> clawPiston1.set(Value.kReverse))
      .andThen(run(() -> clawPiston2.set(Value.kReverse)));
  }

  public CommandBase openCone() {
    return run(() -> clawPiston1.set(Value.kReverse))
      .andThen(run(() -> clawPiston2.set(Value.kForward)));
  }

  public CommandBase closeCone() {
    return run(() -> clawPiston1.set(Value.kForward))
      .andThen(run(() -> clawPiston2.set(Value.kForward)));
  }
}
