package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {

  public SwerveDriveOdometry swerveOdometry;
  public static SwerveModule[] mSwerveMods;
  public XboxController driver = new XboxController(0);

  // private final int translationAxis = XboxController.Axis.kLeftY.value ^ 3;
  // private final int strafeAxis = XboxController.Axis.kLeftX.value ^ 3;
  // private final int rotationAxis = XboxController.Axis.kRightX.value ^ 3;

  public Pigeon2 gyro;
  private GenericEntry gyroAngle;
  Limelight m_Limelight;
  private PIDController m_balancePID = new PIDController(0, 0, 0); //actually get

  public Swerve(Limelight limelight) {
    m_Limelight = limelight;
    gyro = new Pigeon2(Constants.Swerve.pigeonID, "torch");

    gyro.configFactoryDefault();
    zeroGyro();

    ShuffleboardTab tab = Shuffleboard.getTab("GyroFinal");
    gyroAngle =
      tab
        .add(getName(), 0)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kGyro)
        .getEntry();

    mSwerveMods =
      new SwerveModule[] {
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants),
      };

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry =
      new SwerveDriveOdometry(
        Constants.Swerve.swerveKinematics,
        getYaw(),
        getModulePositions()
      );
  }

  public void drive(
    Translation2d translation,
    double rotation,
    boolean fieldRelative,
    boolean isOpenLoop
  ) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          getYaw()
        )
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,
      Constants.Swerve.maxSpeed
    );

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public void autoBalance() {
    m_balancePID.setTolerance(.001);
    double pidOutput;
    pidOutput = MathUtil.clamp(m_balancePID.calculate(getRoll(), 0), -0.4, 0.4);
    drive(new Translation2d(-pidOutput, 0), 0.0, false, true);
  }

  public void xWheels() { //2 1 0 3, BL, FR, FL, BR
    mSwerveMods[0].setDesiredState(
        new SwerveModuleState(0.0, new Rotation2d(45)),
        false
      );

    mSwerveMods[1].setDesiredState(
        new SwerveModuleState(0.0, new Rotation2d(45)),
        false
      );

    mSwerveMods[2].setDesiredState(
        new SwerveModuleState(0.0, new Rotation2d(125)),
        false
      );

    mSwerveMods[3].setDesiredState(
        new SwerveModuleState(0.0, new Rotation2d(125)),
        false
      );
  }

  public void alignToGoal() {
    drive(new Translation2d(0, m_Limelight.getSteeringValue()), 0, true, false);
  }

  // WIP

  // public void alignToCenterClockwise() {
  //   drive(new Translation2d(0, 0), -1, true, false);
  // }

  // public void alignToCenterCounterclockwise() {
  //   drive(new Translation2d(0, 0), 1, true, false);
  // }

  // public CommandBase alignToBase() {
  //   if (Math.abs(getYaw().getDegrees() % 360) >= 180) {
  //     return run(() -> alignToCenterCounterclockwise())
  //       .until(() ->
  //         Math.abs(getYaw().getDegrees() % 360) <= 10 ||
  //         Math.abs(getYaw().getDegrees() % 360) >= 350
  //       );
  //   } else {
  //     return run(() -> alignToCenterClockwise())
  //       .until(() ->
  //         Math.abs(getYaw().getDegrees() % 360) <= 10 ||
  //         Math.abs(getYaw().getDegrees() % 360) >= 350
  //       );
  //   }
  // }

  public CommandBase moveToGoalAprilTags() {
    return runOnce(() -> m_Limelight.setToAprilTags())
      .andThen(() -> Timer.delay(.5))
      .andThen(
        run(() -> alignToGoal())
          .until(() -> m_Limelight.getSteeringValue() == 0)
      );
  }

  public CommandBase moveToGoalRetroreflective() {
    return runOnce(() -> m_Limelight.setToRetroreflectiveTape())
      .andThen(() -> Timer.delay(.5))
      .andThen(
        run(() -> alignToGoal())
          .until(() -> m_Limelight.getSteeringValue() == 0)
      );
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      Constants.Swerve.maxSpeed
    );

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
      ? Rotation2d.fromDegrees(360 - gyro.getYaw())
      : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public static void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());
    SmartDashboard.putNumber("gyro", gyro.getYaw());
    gyroAngle.setDouble(gyro.getYaw());
    //SmartDashboard.putNumber("gyro2", getyaw);
    SmartDashboard.putNumber("yawdegrees", getYaw().getDegrees());
    SmartDashboard.putNumber(
      "yawdegreesmodulo",
      Math.abs(getYaw().getDegrees()) % 360
    );

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Cancoder",
        mod.getCanCoder().getDegrees()
      );
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Integrated",
        mod.getPosition().angle.getDegrees()
      );
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Velocity",
        mod.getState().speedMetersPerSecond
      );
      SmartDashboard.putNumber("X pose", this.getPose().getX());
      SmartDashboard.putNumber("Y pose", this.getPose().getY());
      SmartDashboard.putBoolean("AUTO", DriverStation.isAutonomous());
      SmartDashboard.putBoolean("AUTOenabled", DriverStation.isAutonomousEnabled());
    }
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
  public CommandBase followTrajectoryCommand(
    PathPlannerTrajectory traj,
    boolean isFirstPath
  ) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if (isFirstPath) {
          PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(
            traj,
            DriverStation.getAlliance()
          );
          resetOdometry(transformed.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(
        traj,
        this::getPose, // Pose supplier
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
        this::setModuleStates, // Module states consumer
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        this // Requires this drive subsystem
      )
    );
  }

  public CommandBase driveCommand() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(
      "Drive4Sesny",
      3,
      3
    );

    return runOnce(() -> {
        resetOdometry(trajectory.getInitialPose());
      })
      .andThen(followTrajectoryCommand(trajectory, true))
      .andThen(() -> drive(new Translation2d(0, 0), 0, true, false));
  }
  //want to comment this bc it uses the same variables so just in case

  // public CommandBase createCommandForTrajectory(
  //   PathPlannerTrajectory trajectory
  // ) {
  //   var thetaController = new ProfiledPIDController(
  //     Constants.AutoConstants.kPThetaController,
  //     0,
  //     0,
  //     Constants.AutoConstants.kThetaControllerConstraints
  //   );
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   return new SwerveControllerCommand(
  //     trajectory,
  //     this::getPose,
  //     Constants.Swerve.swerveKinematics,
  //     new PIDController(Constants.AutoConstants.kPXController, 0, 0),
  //     new PIDController(Constants.AutoConstants.kPYController, 0, 0),
  //     thetaController,
  //     this::setModuleStates,
  //     this
  //   );
  // }
}
