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
      clawPiston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
      clawPiston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3); //idk
      clawMotor = new TalonFX(4);
      clawPiston1.set(Value.kForward);
      clawPiston2.set(Value.kForward);
    }
  }

  public CommandBase motorForward() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, .2));
  }

  public CommandBase motorOff() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, 0));
  }

  public CommandBase motorReverse() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, -.2));
  }

  public CommandBase open1In() {
    return runOnce(() -> clawPiston1.set(Value.kForward))
      .andThen(runOnce(() -> clawPiston2.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, .5)));
  }

  public CommandBase open1Hold() {
    return runOnce(() -> clawPiston1.set(Value.kForward))
      .andThen(runOnce(() -> clawPiston2.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  public CommandBase openAllIn() {
    return runOnce(() -> clawPiston2.set(Value.kReverse))
      .andThen(runOnce(() -> clawPiston1.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, .2)));
  }

  public CommandBase openAllHold() {
    return runOnce(() -> clawPiston2.set(Value.kReverse))
      .andThen(runOnce(() -> clawPiston1.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  public CommandBase openAllOut() {
    return runOnce(() -> clawPiston2.set(Value.kReverse))
      .andThen(runOnce(() -> clawPiston1.set(Value.kReverse)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, -.2)));
  }

  public CommandBase closeAllHold() {
    return runOnce(() -> clawPiston1.set(Value.kForward))
      .andThen(runOnce(() -> clawPiston2.set(Value.kForward)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }
  // public CommandBase LOPEN() {
  //   return run(() -> clawPiston1.set(Value.kForward)); //closes L
  // }

  // public CommandBase LCLOSE() {
  //   return run(() -> clawPiston1.set(Value.kReverse)); //opens L
  // }

  // public CommandBase ROPEN() {
  //   return run(() -> clawPiston2.set(Value.kForward)); //closes R
  // }

  // public CommandBase RCLOSE() {
  //   return run(() -> clawPiston2.set(Value.kReverse)); //opens R
  // }
}
