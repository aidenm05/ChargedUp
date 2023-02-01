package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Fourbar extends SubsystemBase {

  public WPI_TalonFX mainMotor;
  public WPI_TalonFX followerMotor;

  public Fourbar() {
    mainMotor = new WPI_TalonFX(3);
    followerMotor = new WPI_TalonFX(4);

    mainMotor.configFactoryDefault();
    followerMotor.configFactoryDefault();
    followerMotor.setInverted(TalonFXInvertType.CounterClockwise);
    mainMotor.setInverted(TalonFXInvertType.Clockwise);
    mainMotor.setNeutralMode(NeutralMode.Brake); // maybe?
    followerMotor.setNeutralMode(NeutralMode.Brake);
    mainMotor.configNeutralDeadband(0.001);
  }
}
