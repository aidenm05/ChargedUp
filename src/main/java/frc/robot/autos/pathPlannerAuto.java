package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class pathPlannerAuto extends SequentialCommandGroup {

  public pathPlannerAuto(
    frc.robot.subsystems.Swerve s_Swerve,
    Elevator m_Elevator,
    Claw m_Claw
  ) {
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    )
      .setKinematics(Constants.Swerve.swerveKinematics);
    PathPlannerTrajectory traj = PathPlanner.loadPath("Drive4Sesny", 2, 2);
    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(() ->
          m_Elevator.setPosition(Constants.elevatorPos1, Constants.armPos1)
        ),
        new WaitCommand(1),
        new InstantCommand(() -> m_Claw.openAllOut()),
        new InstantCommand(() -> s_Swerve.followTrajectoryCommand(traj, true))
      )
    );
  }
}
