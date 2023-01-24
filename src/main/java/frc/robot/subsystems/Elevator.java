package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  public WPI_TalonFX mainMotor;
  public WPI_TalonFX followerMotor;
  public double calculatedPosition = 0;
  public double motorPosition;
  StringBuilder _sb = new StringBuilder();
  public int smoothing = 0;
  int targetPos = 10000;


  public Elevator() {
    mainMotor = new WPI_TalonFX(1); // add "torch as second parameter when on canivore"
    followerMotor = new WPI_TalonFX(2); // add "torch as second parameter when on canivore"
    mainMotor.configFactoryDefault();
    followerMotor.configFactoryDefault();
    mainMotor.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      0,
      30
    );

    followerMotor.follow(mainMotor); //set the follower motor to mimic the mainmotor

    followerMotor.setInverted(TalonFXInvertType.Clockwise); // motors need to be inverted from each other as they face opposite ways.  We need to determine if positive is up or down on the elevator.
    mainMotor.setInverted(TalonFXInvertType.CounterClockwise);

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
    mainMotor.config_kP(Constants.kSlotIdx, .1, Constants.kTimeoutMs);
    mainMotor.config_kI(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);
    mainMotor.config_kD(Constants.kSlotIdx, 0.0, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    mainMotor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    mainMotor.configMotionAcceleration(15000, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    mainMotor.setSelectedSensorPosition(
      0,
      Constants.kPIDLoopIdx,
      Constants.kTimeoutMs
    );
  }

  public void runUp() {
    mainMotor.set(TalonFXControlMode.PercentOutput, 1);
  }

  public void runDown() {
    mainMotor.set(TalonFXControlMode.PercentOutput, -0.2);
  }

  public void stop() {
    mainMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  //Attempt to use triggers
  //public CommandBase runUp(){
  //    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, .5))
  //            .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
  //            .withName("runUp");
  //}

  //public CommandBase runDown(){
  //    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, -.5))
  //            .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.0))
  //            .withName("runDown");
  //}

  public void increasePosition() {
    calculatedPosition = calculatedPosition + 1;
    //mainMotor.set(TalonFXControlMode.Position, calculatedPosition)
  }

  public void decreasePosition() {
    calculatedPosition = calculatedPosition - 1;
    //mainMotor.set(TalonFXControlMode.Position, calculatedPosition);
  }

  public void setPosition() {
    
    mainMotor.set(TalonFXControlMode.MotionMagic, targetPos);
  }

  public void reset() {
    mainMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator", mainMotor.getSelectedSensorPosition());
    double motorOutput = mainMotor.getMotorOutputPercent();
    
    _sb.append("\tOut%");
    _sb.append(motorOutput);
    _sb.append("\tVel");
    _sb.append(mainMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

    _sb.append("\terr;");
    _sb.append(mainMotor.getClosedLoopError(Constants.kPIDLoopIdx));
    _sb.append("\ttrg:");
    _sb.append(targetPos);

    

  }


}
