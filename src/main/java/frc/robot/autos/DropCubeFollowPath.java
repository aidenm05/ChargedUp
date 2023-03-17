package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class DropCubeFollowPath extends SequentialCommandGroup {

  public DropCubeFollowPath(
    Swerve s_Swerve,
    Elevator m_Elevator,
    Claw m_Claw,
    int EP,
    int AP,
    String path,
    boolean runBalancer
  ) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(path, 1.5, 2);
    addCommands(
      m_Claw.open1Hold(),
      m_Elevator.sequentialSetPositions(EP, AP),
      // new WaitCommand(2),
      m_Claw.openAllOut(),
      new WaitCommand(.5),
      m_Claw.motorOff(),
      m_Elevator.setStow(),
      // new WaitCommand(.5), // If running too quickly, add back in
      s_Swerve.followTrajectoryCommand(traj, true)
    );
    if (runBalancer) {
      addCommands(
        new RunCommand(s_Swerve::autoBalance, s_Swerve),
        s_Swerve.xWheelsCommand()
      );
    }
  }
}
