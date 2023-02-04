package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class Arm extends SubsystemBase {

  public WPI_TalonFX armMotor;
  public Elevator elevator;

  public Arm() {
    SupplyCurrentLimitConfiguration elevatorSupplyLimit = new SupplyCurrentLimitConfiguration(
      true,
      25,
      40,
      .1
    );
    if (!Constants.mantis) {
      armMotor = new WPI_TalonFX(3, "torch");

      armMotor.setNeutralMode(NeutralMode.Brake);
      armMotor.configNeutralDeadband(.001);
      armMotor.configSupplyCurrentLimit(elevatorSupplyLimit);
      armMotor.setInverted(true);

      armMotor.configForwardSoftLimitEnable(false);
      armMotor.configForwardSoftLimitThreshold(30000);
      armMotor.configReverseSoftLimitEnable(false);
      armMotor.configReverseSoftLimitThreshold(-9000);
    }
  }

  public CommandBase resetArmEncoder() {
    return run(() -> armMotor.setSelectedSensorPosition(0));
  }

  //basic percent outputs for arm
  public CommandBase armDown() {
    return run(() -> armMotor.set(TalonFXControlMode.PercentOutput, -.2))
      .finallyDo(interrupted -> armMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("armDown");
  }

  public CommandBase armUp() {
    return run(() -> armMotor.set(TalonFXControlMode.PercentOutput, .2))
      .finallyDo(interrupted -> armMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("armUp");
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Elevator From Arm", elevator.getPosition());
  }
}
