package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  public final GenericHID operator = new GenericHID(0);

 // Trigger exampleTrigger = new Trigger(() -> operator.getRawAxis(1) > 0.5
  //);
  /* Driver Buttons */
  private final JoystickButton yButton1 = new JoystickButton(
    operator,
    XboxController.Button.kY.value
  );
  private final JoystickButton leftBumper1 = new JoystickButton( //maybe we wanna change this to leftBumper
    operator,
    XboxController.Button.kLeftBumper.value
  );
  private final JoystickButton aButton1 = new JoystickButton(
    operator,
    XboxController.Button.kA.value
  );
  private final JoystickButton bButton1 = new JoystickButton(
    operator,
    XboxController.Button.kB.value
  );
  private final JoystickButton rightBumper1 = new JoystickButton( //rightBumper
    operator,
    XboxController.Button.kRightBumper.value
  );
  private final JoystickButton xButton1 = new JoystickButton(
    operator,
    XboxController.Button.kX.value
  );
 
  private final JoystickButton leftStickButton1 = new JoystickButton(
    operator,
    XboxController.Button.kLeftStick.value
  );

  private final JoystickButton rightStickButton1 = new JoystickButton(
    operator,
    XboxController.Button.kRightStick.value
  );

  private final POVButton b7 = new POVButton(operator, 0);

  private final POVButton b8 = new POVButton(operator, 90);

  private final POVButton b9 = new POVButton(operator, 180);

  private final POVButton b10 = new POVButton(operator, 270);

  // private final JoystickButton aButton2 = new JoystickButton(
  //   operator2,
  //   XboxController.Button.kA.value
  // );
  // private final JoystickButton bButton2 = new JoystickButton(
  //   operator2,
  //   XboxController.Button.kB.value
  // );
  // private final JoystickButton xButton2 = new JoystickButton(
  //   operator2,
  //   XboxController.Button.kX.value
  // );
  // private final JoystickButton yButton2 = new JoystickButton(
  //   operator2,
  //   XboxController.Button.kY.value
  // );

  // private final JoystickButton leftBumper2 = new JoystickButton(
  //   operator2,
  //   XboxController.Button.kLeftBumper.value
  // );

  // private final JoystickButton rightBumper2 = new JoystickButton(
  //   operator2,
  //   XboxController.Button.kRightBumper.value
  // );

  private final JoystickButton start = new JoystickButton(
    operator,
    XboxController.Button.kStart.value
  );

  private final JoystickButton back = new JoystickButton(
    operator,
    XboxController.Button.kBack.value
  );

  //final JoystickButton b1 = new JoystickButton(buttonBoard, 1);
 // final JoystickButton b2 = new JoystickButton(buttonBoard, 2);
 // final JoystickButton b3 = new JoystickButton(buttonBoard, 3);
//  final JoystickButton b4 = new JoystickButton(buttonBoard, 4);
 // final JoystickButton b5 = new JoystickButton(buttonBoard, 5);
  //final JoystickButton b6 = new JoystickButton(buttonBoard, 6);
  // final JoystickButton b7 = new JoystickButton(buttonBoard, 7);
  // final JoystickButton b8 = new JoystickButton(buttonBoard, 8);
  // final JoystickButton b9 = new JoystickButton(buttonBoard, 9);
  // final JoystickButton b10 = new JoystickButton(buttonBoard, 10);
  //final JoystickButton b11 = new JoystickButton(buttonBoard, 11);
 // final JoystickButton b12 = new JoystickButton(buttonBoard, 12);
//  Trigger bbStickF = new Trigger(() -> buttonBoard.getRawAxis(1) > 0.7);
 // Trigger bbStickB = new Trigger(() -> buttonBoard.getRawAxis(1) < -0.7);

  /* Subsystems */
  private final Limelight m_Limelight = new Limelight();
  public final Swerve s_Swerve = new Swerve(m_Limelight);
  public final Elevator m_Elevator = new Elevator();
  private final Claw m_Claw = new Claw();
  //private final LED m_LED = new LED();

  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Swerve.resetModulesToAbsolute();
    //Swerve.resetModulesToAbsolute();

    m_autoChooser.setDefaultOption("Nothing", new InstantCommand());

    m_autoChooser.addOption(
      "Leave Community",
      new LeaveCommunityAuto(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "Cone Then Cube",
      new ConeThenCubeSimple(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "Cone Then Cone",
      new ConeThenConeSimple(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "Test Left Auto",
      new TestLeftAuto(s_Swerve, m_Elevator, m_Claw)
    );

    m_autoChooser.addOption(
      "Top Cone Charge Balance",
      new DropConeFollowPath(
        s_Swerve,
        m_Elevator,
        m_Claw,
        Constants.elevatorTopCone,
        Constants.armTopCone,
        "GPWithCharge",
        true
      )
    );

    m_autoChooser.addOption(
      "Top Cone Mobility Balance",
      new DropConeFollowPath(
        s_Swerve,
        m_Elevator,
        m_Claw,
        Constants.elevatorTopCone,
        Constants.armTopCone,
        "GPMobilityCharge",
        true
      )
    );

    m_autoChooser.addOption(
      "Top Cone Then Grab Piece",
      new DropConeFollowPath(
        s_Swerve,
        m_Elevator,
        m_Claw,
        Constants.elevatorTopCone,
        Constants.armTopCone,
        "Cone2GP",
        false
      )
    );

    // m_autoChooser.addOption(
    //   "Cone Then Cube",
    //   new DropConeFollowPath(
    //     s_Swerve,
    //     m_Elevator,
    //     m_Claw,
    //     Constants.elevatorTopCone,
    //     Constants.armTopCone,
    //     "ConeThenCube",
    //     true
    //   )
    // );

    m_autoChooser.addOption(
      "Top Cube Charge Balance",
      new DropCubeFollowPath(
        s_Swerve,
        m_Elevator,
        m_Claw,
        Constants.elevatorTopCube,
        Constants.armTopCube,
        "GPWithCharge",
        true
      )
    );

    m_autoChooser.addOption(
      "Top Cone Leave Community",
      new DropConeFollowPath(
        s_Swerve,
        m_Elevator,
        m_Claw,
        Constants.elevatorTopCone,
        Constants.armTopCone,
        "Leave",
        false
      )
    );

    m_autoChooser.addOption(
      "Mid Cone Leave Community",
      new DropConeFollowPath(
        s_Swerve,
        m_Elevator,
        m_Claw,
        Constants.elevatorMidCone,
        Constants.armMidCone,
        "Leave",
        false
      )
    );

    m_autoChooser.addOption(
      "Top Cube Leave Community",
      new DropCubeFollowPath(
        s_Swerve,
        m_Elevator,
        m_Claw,
        Constants.elevatorTopCube,
        Constants.armTopCube,
        "Leave",
        false
      )
    );

    m_autoChooser.addOption(
      "Mid Cube Leave Community",
      new DropCubeFollowPath(
        s_Swerve,
        m_Elevator,
        m_Claw,
        Constants.elevatorMidCube,
        Constants.armMidCube,
        "Leave",
        false
      )
    );

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    m_Elevator.armAndElevatorStopPercentMode();

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    if (!Constants.mantis) {
      leftBumper1.whileTrue(m_Claw.motorForward());
      leftBumper1.onFalse(m_Claw.open1Hold());
      rightBumper1.whileTrue(m_Claw.motorReverse());
      rightBumper1.onFalse(m_Claw.closeAllHold());

      // leftStickButton1.onTrue(new ConditionalCommand(m_Elevator.setStow(), m_Elevator.sequentialSetPositions(
      //   Constants.elevatorFloor,
      //   Constants.armFloor
      // ), leftStickButton1.))

      // aButton1.onTrue(m_Elevator.setPositions(205800, 976));
      // bButton1.onTrue(m_Elevator.setPositions(80000, 1000));

      // xButton1.onTrue(s_Swerve.moveToGoalRetroreflective());

      // yButton1.onTrue(s_Swerve.moveToGoalAprilTags());

      // xButton1.whileTrue(new RunCommand(s_Swerve::autoBalance, s_Swerve));
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
//  sus.onTrue(s_Swerve.xWheelsCommand());
//--------------------------------------------------------------------------------------------------------------------------------------------------------------

      aButton1.onTrue(
        m_Elevator.sequentialSetPositions(
          Constants.elevatorFloor,
          Constants.armFloor
        )
      );

      bButton1.onTrue(m_Elevator.setStow());

      //Elevator Arm Presets
      xButton1.onTrue(
        m_Elevator.sequentialSetPositions(
          Constants.elevatorTopCone,
          Constants.armTopCone
        )
      );
      yButton1.onTrue(
        m_Elevator.sequentialSetPositions(
          Constants.elevatorMidCone,
          Constants.armMidCone
        )
      );
      start.onTrue(
        m_Elevator.sequentialSetPositions(
          Constants.elevatorTopCube,
          Constants.armTopCube
        )
      );
      back.onTrue(
        m_Elevator.sequentialSetPositions(
          Constants.elevatorMidCube,
          Constants.armMidCube
        )
      );
      rightStickButton1.onTrue(
        m_Elevator.sequentialSetPositions(
          Constants.elevatorShelf,
          Constants.armShelf
        )
      );

      //dpad
      b7.whileTrue(m_Elevator.armUp());
      b8.whileTrue(m_Elevator.armDown());
      b9.whileTrue(m_Elevator.runUp());
      b10.whileTrue(m_Elevator.runDown());




//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
      // dUp1.whileTrue(
      //   s_Swerve.driveContinuous(new Translation2d(.2, 0), 0, true, false)
      // );

      // dRight1.whileTrue(
      //   s_Swerve.driveContinuous(new Translation2d(0, -0.2), 0, true, false)
      // );

      // dDown1.whileTrue(
      //   s_Swerve.driveContinuous(new Translation2d(-0.2, 0), 0, true, false)
      // );

      // dLeft1.whileTrue(
      //   s_Swerve.driveContinuous(new Translation2d(0, .2), 0, true, false)
      // );
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public class LeftTriggerPressed extends JoystickButton {

    public LeftTriggerPressed(GenericHID joystick, int buttonNumber) {
      super(joystick, buttonNumber);
      //TODO Auto-generated constructor stub
    }
    // @Override
    // public boolean getAsBoolean() {
    //   return operator.getRawAxis(2) < -0.5;
    //   // This returns whether the trigger is active
    // }
  }
  // public class RightTriggerPressed extends Trigger {

  //   @Override
  //   public boolean getAsBoolean() {
  //     return operator.getRawAxis(3) > 0.5;
  //     // This returns whether the trigger is active
  //   }
  // }
}
