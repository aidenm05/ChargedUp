package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  public Solenoid clawPiston;

  public Claw() {
    if (!Constants.mantis) {
      clawPiston = new Solenoid(1, PneumaticsModuleType.REVPH, 0);
    }
  }

  public CommandBase close() {
    return run(() -> clawPiston.set(false));
  }

  public CommandBase open() {
    return run(() -> clawPiston.set(true));
  }
}
