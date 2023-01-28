package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  public WPI_TalonFX mainMotor;
  public WPI_TalonFX followerMotor;
  public double calculatedPOutput = 0;
  public double motorPosition;
  StringBuilder _sb = new StringBuilder();
  public int smoothing = 0;
  int upTargetPos = 10000;
  int downTargetPosition = 100;
  int count = 0;

  public Elevator() {
    mainMotor = new WPI_TalonFX(1, "torch"); // add "torch as second parameter when on canivore"
    followerMotor = new WPI_TalonFX(2, "torch"); // add "torch as second parameter when on canivore"
    mainMotor.configFactoryDefault();
    followerMotor.configFactoryDefault();
    mainMotor.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      0,
      30
    );

    followerMotor.follow(mainMotor); //set the follower motor to mimic the mainmotor

    followerMotor.setInverted(TalonFXInvertType.CounterClockwise); // motors need to be inverted from each other as they face opposite ways.  We need to determine if positive is up or down on the elevator.
    mainMotor.setInverted(TalonFXInvertType.Clockwise);
    mainMotor.setNeutralMode(NeutralMode.Brake);
    followerMotor.setNeutralMode(NeutralMode.Brake);
    mainMotor.configNeutralDeadband(0.001);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    mainMotor.setStatusFramePeriod(
      StatusFrameEnhanced.Status_13_Base_PIDF0,
      10,
      Constants.kTimeoutMs
    );
    mainMotor.setStatusFramePeriod(
      StatusFrameEnhanced.Status_10_MotionMagic,
      10,
      Constants.kTimeoutMs
    );

    /* Set the peak and nominal outputs */
    mainMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    mainMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    mainMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    mainMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    mainMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    mainMotor.config_kF(Constants.kSlotIdx, 1000, Constants.kTimeoutMs);
    mainMotor.config_kP(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);
    mainMotor.config_kI(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);
    mainMotor.config_kD(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    mainMotor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    mainMotor.configMotionAcceleration(15000, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up not forever */
    mainMotor.setSelectedSensorPosition(
      0,
      Constants.kPIDLoopIdx,
      Constants.kTimeoutMs
    );
  }

  // public void runUp() {
  //   mainMotor.set(TalonFXControlMode.PercentOutput, 1);
  // }

  // public void runDown() {
  //   mainMotor.set(TalonFXControlMode.PercentOutput, -0.2);
  // }

  // public void stop() {
  //   mainMotor.set(TalonFXControlMode.PercentOutput, 0);
  // }

  //nice run up and down commands
  public CommandBase runDown() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, -.5))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("runDown");
  }

  public CommandBase runUp() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, .5))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
      .withName("runDown");
  }

  //methods to find percent outputs needed for feedforward etc etc
  public void increasePercentOutput() {
    calculatedPOutput = calculatedPOutput + .05;
    goTo(calculatedPOutput);
  }

  public void decreasePercentOutput() {
    calculatedPOutput = calculatedPOutput - .05;
    goTo(calculatedPOutput);
  }

  public void goTo(double calculatedPercentOutput) {
    mainMotor.set(ControlMode.PercentOutput, calculatedPercentOutput);
  }

  //motion magic shenanigans
  public void setupPosition() {
    mainMotor.set(TalonFXControlMode.MotionMagic, upTargetPos);
  }

  public void setDownPosition() {
    mainMotor.set(TalonFXControlMode.MotionMagic, downTargetPosition);
  }

  public void reset() {
    mainMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator", mainMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Percent Output", calculatedPOutput);
    double motorOutput = mainMotor.getMotorOutputPercent();

    _sb.append("\nOut%");
    _sb.append(motorOutput);
    _sb.append("\t Vel");
    _sb.append(mainMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

    _sb.append("\t Position:");
    _sb.append(mainMotor.getSelectedSensorPosition());

    _sb.append("\t err;");
    _sb.append(mainMotor.getClosedLoopError(Constants.kPIDLoopIdx));
    _sb.append("\t trg:");
    _sb.append(upTargetPos);
    if (count % 20 == 0) {
      count = 0;
      System.out.println(_sb.toString());
    }
    count = count + 1;
    _sb.setLength(0);
  }
}
