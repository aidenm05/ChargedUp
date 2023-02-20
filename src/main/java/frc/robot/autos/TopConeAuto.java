package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class TopConeAuto extends SequentialCommandGroup {

  public TopConeAuto(Swerve s_Swerve, Elevator m_Elevator, Claw m_Claw) {
    PathPlannerTrajectory traj = PathPlanner.loadPath("Leave", 2, 2);
    addCommands(
      m_Elevator.setPosition(Constants.elevatorTopCone, Constants.armTopCone),
      new WaitCommand(2),
      //add claw open
      m_Elevator.setPosition(Constants.elevatorStow, Constants.armStow),
      new WaitCommand(2),
      s_Swerve.followTrajectoryCommand(traj, true)
    );
  }
}
