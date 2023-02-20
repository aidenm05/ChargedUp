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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class ExampleAuto extends SequentialCommandGroup {

  public ExampleAuto(
    Swerve s_Swerve,
    Elevator m_Elevator,
    Claw m_Claw,
    int EP,
    int AP,
    String path
  ) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(path, 2, 2);
    addCommands(
      m_Elevator.sequentialSetPositions(EP, AP),
      new WaitCommand(2),
      //add claw open
      // wait
      m_Elevator.setStow(),
      new WaitCommand(2),
      s_Swerve.followTrajectoryCommand(traj, true)
    );
  }
}
