package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class ConeThenCube extends SequentialCommandGroup {

  public ConeThenCube(Swerve s_Swerve, Elevator m_Elevator, Claw m_Claw) {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Cone2GP", 2, 2);
    PathPlannerTrajectory traj2 = PathPlanner.loadPath("GPGet", .5, 2);
    PathPlannerTrajectory traj3 = PathPlanner.loadPath("GP2Cube", 2, 2);
    addCommands(
      m_Claw.closeAllHold(),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCone,
        Constants.armTopCone
      ),
      m_Claw.openAllOut(),
      new WaitCommand(.5),
      m_Claw.motorOff(),
      m_Elevator.setStow(),
      // new WaitCommand(.5), // If running too quickly, add back in
      s_Swerve.followTrajectoryCommand(traj1, true),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorFloor,
        Constants.armFloor
      ),
      m_Claw.openAllIn(),
      s_Swerve.followTrajectoryCommand(traj2, false),
      m_Claw.open1Hold(),
      m_Elevator.setStow(),
      s_Swerve.followTrajectoryCommand(traj3, false),
      m_Elevator.sequentialSetPositions(
        Constants.elevatorTopCube,
        Constants.armTopCube
      ),
      m_Claw.openAllOut(),
      new WaitCommand(.5),
      m_Claw.motorOff(),
      m_Elevator.setStow()
    );
  }
}
