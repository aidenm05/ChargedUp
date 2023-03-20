package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  double tx = 0;
  double ty = 0;
  double tv = 0;
  double ta = 0;
  private SendableChooser<Boolean> m_limelightSwitch = new SendableChooser<>();

  public Limelight() {
    m_limelightSwitch.setDefaultOption("On", true);
    m_limelightSwitch.addOption("Off", false);

    SmartDashboard.putData("Limelight Switch", m_limelightSwitch);
  }

  //rotation for vision tracking-- we need to correct rotation on the limelight

  @Override
  public void periodic() {
    tx =
      NetworkTableInstance
        .getDefault()
        .getTable("limelight")
        .getEntry("tx")
        .getDouble(0);
    SmartDashboard.putNumber("tx", tx);

    ty =
      NetworkTableInstance
        .getDefault()
        .getTable("limelight")
        .getEntry("ty")
        .getDouble(0);
    SmartDashboard.putNumber("ty", ty);

    tv =
      NetworkTableInstance
        .getDefault()
        .getTable("limelight")
        .getEntry("tv")
        .getDouble(0);
    SmartDashboard.putBoolean("tv", tv >= 1.0);

    ta =
      NetworkTableInstance
        .getDefault()
        .getTable("limelight")
        .getEntry("ta")
        .getDouble(0);
    SmartDashboard.putBoolean("target valid", ta >= 1.0);

    SmartDashboard.putNumber("getSteeringValue", getSteeringValue());
  }

  public void setToAprilTags() {
    NetworkTableInstance
      .getDefault()
      .getTable("limelight")
      .getEntry("pipeline")
      .setNumber(0.0);
  }

  public void setToRetroreflectiveTape() {
    NetworkTableInstance
      .getDefault()
      .getTable("limelight")
      .getEntry("pipeline")
      .setNumber(1.0);
  }

  public double getSteeringValue() {
    double STEER_K = 0.1;

    double signumtx = Math.signum(tx);

    // if tv = 0, target is not valid so return 0.0
    if (tv == 0) {
      return 0.0;
    }

    if (m_limelightSwitch.getSelected() == false) {
      return 0.0;
    }

    double txAbs = Math.abs(tx);
    double txDeadband = txAbs - Constants.LIMELIGHT_DEADBAND;

    if (txDeadband < 0) {
      return 0.0;
    }

    double minDriveWithSine = signumtx * Constants.MIN_STEER_K;
    double steer_cmd = tx * STEER_K;
    double finalSteerCmd = minDriveWithSine + steer_cmd;

    return finalSteerCmd;
  }
}
