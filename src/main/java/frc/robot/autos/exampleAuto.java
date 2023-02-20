package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class exampleAuto extends SequentialCommandGroup {

  public exampleAuto(Swerve s_Swerve, Elevator m_Elevator, Claw m_Claw) {
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    )
      .setKinematics(Constants.Swerve.swerveKinematics);

    PathPlannerTrajectory traj = PathPlanner.loadPath("Drive4Sesny", 2, 2);

    s_Swerve.followTrajectoryCommand(traj, true);
    // addCommands(
    //   s_Swerve.followTrajectoryCommand(traj, true)(traj, true);
    // )
  }
}
