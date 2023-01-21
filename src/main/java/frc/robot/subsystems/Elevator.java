package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import javax.swing.text.Position;

public class Elevator extends SubsystemBase {

  public WPI_TalonFX mainMotor;
  public WPI_TalonFX followerMotor;
  public double targetVelocity;

  public Elevator() {
    mainMotor = new WPI_TalonFX(1); // change
    followerMotor = new WPI_TalonFX(2); // change

    mainMotor.configFactoryDefault();
    followerMotor.configFactoryDefault();
    // 491,717
    mainMotor.configForwardSoftLimitThreshold(3.0);

    followerMotor.follow(mainMotor);
    followerMotor.setInverted(TalonFXInvertType.Clockwise); // subject to change
    mainMotor.setInverted(TalonFXInvertType.CounterClockwise);

    mainMotor.configNeutralDeadband(0.01);
  }

  public void runUp() {
    mainMotor.set(TalonFXControlMode.PercentOutput, .5);
  }

  public void runDown() {
    mainMotor.set(TalonFXControlMode.PercentOutput, -.5);
  }

  public void reset() {
    mainMotor.set(TalonFXControlMode.Position, 0);
  }

  @Override
  public void periodic() {
    double position = mainMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("sensor position", position);
  }
}
