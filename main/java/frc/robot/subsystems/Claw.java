package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  public Solenoid clawPiston1;
  public Solenoid clawPiston2;
  public TalonFX clawMotor;

  public Claw() {
    
      clawPiston1 = new Solenoid(PneumaticsModuleType.REVPH, 1);
      clawPiston2 = new Solenoid(PneumaticsModuleType.REVPH, 2); //idk
      clawMotor = new TalonFX(4);
      clawPiston1.set(true);
      clawPiston2.set(false);
    
  }

  public CommandBase motorForward() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, .2));
  }

  public CommandBase motorOff() {
    return runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0));
  }

  public CommandBase motorReverse() {
    return run(() -> clawMotor.set(ControlMode.PercentOutput, -.2));
  }

  public CommandBase open1In() {
    return runOnce(() -> clawPiston1.set(true))
      .andThen(runOnce(() -> clawPiston2.set(false)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, .5)));
  }

  public CommandBase open1Hold() {
    return runOnce(() -> clawPiston1.set(true))
      .andThen(runOnce(() -> clawPiston2.set(true)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  public CommandBase openAllIn() {
    return runOnce(() -> clawPiston2.set(true))
      .andThen(runOnce(() -> clawPiston1.set(true)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, .2)));
  }

  public CommandBase openAllHold() {
    return runOnce(() -> clawPiston2.set(true))
      .andThen(runOnce(() -> clawPiston1.set(true)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  public CommandBase openAllOut() {
    return runOnce(() -> clawPiston2.set(true))
      .andThen(runOnce(() -> clawPiston1.set(true)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, -.2)));
  }

  public CommandBase openAllDrop() {
    return runOnce(() -> clawPiston2.set(true))
      .andThen(runOnce(() -> clawPiston1.set(true)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }

  public CommandBase closeAllHold() {
    return runOnce(() -> clawPiston1.set(true))
      .andThen(runOnce(() -> clawPiston2.set(true)))
      .andThen(runOnce(() -> clawMotor.set(ControlMode.PercentOutput, 0)));
  }
  // public CommandBase LOPEN() {
  //   return run(() -> clawPiston1.set(true)); //closes L
  // }

  // public CommandBase LCLOSE() {
  //   return run(() -> clawPiston1.set(true)); //opens L
  // }

  // public CommandBase ROPEN() {
  //   return run(() -> clawPiston2.set(true)); //closes R
  // }

  // public CommandBase RCLOSE() {
  //   return run(() -> clawPiston2.set(true)); //opens R
  // }
}
