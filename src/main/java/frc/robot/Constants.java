package frc.robot;

//hello
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final boolean mantis = true;
  public static final double LIMELIGHT_DEADBAND = 1;
  public static final double MIN_STEER_K = .4;

  // weird motion magic
  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from
   * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
   * configuration.
   */
  public static final int kSlotIdx = 0;
  public final int test = 0;
  public static final int kPIDLoopIdx = 0;
  public static int kTimeoutMs = 30;

  // /**
  //  * Gains used in Motion Magic, to be adjusted accordingly
  //    * Gains(kp, ki, kd, kf, izone, peak output);
  //    */

  public static final double stickDeadband = 0.1;

  public static final class Swerve {

    public static int pigeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i( //TODO: This must be tuned to specific robot
      COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1
    );

    /* Drivetrain Constants */

    public static double trackWidth = Units.inchesToMeters(19.75); //TODO: This must be tuned to specific robot
    public static double wheelBase = Units.inchesToMeters(28.5); //TODO: This must be tuned to specific robot

    public static double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    /* Module Gear Ratios */
    public static double driveGearRatio = chosenModule.driveGearRatio;
    public static double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot

      public static int driveMotorID = 12;
      public static int angleMotorID = 11;
      public static int canCoderID = 10;
      public static Rotation2d angleOffset = Rotation2d.fromDegrees(191.6);
      public static SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot

      public static int driveMotorID = 42;
      public static int angleMotorID = 41;
      public static int canCoderID = 40;
      public static Rotation2d angleOffset = Rotation2d.fromDegrees(304.98);
      public static SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot

      public static int driveMotorID = 22;
      public static int angleMotorID = 21;
      public static int canCoderID = 20;
      public static Rotation2d angleOffset = Rotation2d.fromDegrees(46.49);
      public static SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot

      public static int driveMotorID = 32;
      public static int angleMotorID = 31;
      public static int canCoderID = 30;
      public static Rotation2d angleOffset = Rotation2d.fromDegrees(43.06);
      public static SwerveModuleConstants constants = new SwerveModuleConstants(
        driveMotorID,
        angleMotorID,
        canCoderID,
        angleOffset
      );
    }
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =
      Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );
  }

  public static void confirmMantisConstants() {
    if (mantis) {
      Swerve.trackWidth = Units.inchesToMeters(19.75);
      Swerve.wheelBase = Units.inchesToMeters(28.5);
      Swerve.pigeonID = 1;
      kTimeoutMs = 10;
      Swerve.wheelCircumference = Swerve.chosenModule.wheelCircumference;
      Swerve.swerveKinematics =
        new SwerveDriveKinematics(
          new Translation2d(Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0),
          new Translation2d(Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0),
          new Translation2d(-Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0),
          new Translation2d(-Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0)
        );

      Swerve.chosenModule =
        COTSFalconSwerveConstants.SDSMK3( //TODO: This must be tuned to specific robot
          COTSFalconSwerveConstants.driveGearRatios.SDSMK3_Fast // or standard idk?
        );
      Swerve.driveGearRatio = Swerve.chosenModule.driveGearRatio;
      Swerve.angleGearRatio = Swerve.chosenModule.angleGearRatio;
      Swerve.driveMotorInvert = Swerve.chosenModule.driveMotorInvert;
      Swerve.angleMotorInvert = Swerve.chosenModule.angleMotorInvert;
      Swerve.canCoderInvert = Swerve.chosenModule.canCoderInvert;
      Swerve.Mod0.angleOffset = Rotation2d.fromDegrees(264.28); //166.99
      Swerve.Mod0.constants =
        new SwerveModuleConstants(
          Swerve.Mod0.driveMotorID,
          Swerve.Mod0.angleMotorID,
          +Swerve.Mod0.canCoderID,
          Swerve.Mod0.angleOffset
        );

      Swerve.Mod1.angleOffset = Rotation2d.fromDegrees(93.16); //275.27
      Swerve.Mod1.constants =
        new SwerveModuleConstants(
          Swerve.Mod1.driveMotorID,
          Swerve.Mod1.angleMotorID,
          Swerve.Mod1.canCoderID,
          Swerve.Mod1.angleOffset
        );

      Swerve.Mod2.angleOffset = Rotation2d.fromDegrees(125.59); //83.84
      Swerve.Mod2.constants =
        new SwerveModuleConstants(
          Swerve.Mod2.driveMotorID,
          Swerve.Mod2.angleMotorID,
          Swerve.Mod2.canCoderID,
          Swerve.Mod2.angleOffset
        );

      Swerve.Mod3.angleOffset = Rotation2d.fromDegrees(345.41); //308.5
      Swerve.Mod3.constants =
        new SwerveModuleConstants(
          Swerve.Mod3.driveMotorID,
          Swerve.Mod3.angleMotorID,
          Swerve.Mod3.canCoderID,
          Swerve.Mod3.angleOffset
        );
    }
  }
}
