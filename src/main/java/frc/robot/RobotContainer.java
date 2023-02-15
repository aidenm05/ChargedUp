package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.lang.reflect.InaccessibleObjectException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final Joystick driver2 = new Joystick(0);
  private final XboxController driver1 = new XboxController(1);
  private final Joystick buttonBoard = new Joystick(1); //maybe 2 idk fix the ports
  Trigger exampleTrigger = new Trigger(() -> driver1.getLeftTriggerAxis() > 0.5
  );
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton back1 = new JoystickButton(
    driver1,
    XboxController.Button.kBack.value
  );
  private final JoystickButton yButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kY.value
  );
  private final JoystickButton leftBumper1 = new JoystickButton( //maybe we wanna change this to leftBumper
    driver1,
    XboxController.Button.kLeftBumper.value
  );
  private final JoystickButton aButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kA.value
  );
  private final JoystickButton bButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kB.value
  );
  private final JoystickButton rightBumper1 = new JoystickButton( //rightBumper
    driver1,
    XboxController.Button.kRightBumper.value
  );
  private final JoystickButton xButton1 = new JoystickButton(
    driver1,
    XboxController.Button.kX.value
  );
  private final JoystickButton start1 = new JoystickButton(
    driver1,
    XboxController.Button.kStart.value
  );

  private final JoystickButton aButton2 = new JoystickButton(
    driver2,
    XboxController.Button.kA.value
  );
  private final JoystickButton bButton2 = new JoystickButton(
    driver2,
    XboxController.Button.kB.value
  );
  private final JoystickButton xButton2 = new JoystickButton(
    driver2,
    XboxController.Button.kX.value
  );
  private final JoystickButton yButton2 = new JoystickButton(
    driver2,
    XboxController.Button.kY.value
  );

  private final JoystickButton leftBumper2 = new JoystickButton(
    driver2,
    XboxController.Button.kLeftBumper.value
  );

  private final JoystickButton rightBumper2 = new JoystickButton(
    driver2,
    XboxController.Button.kRightBumper.value
  );

  private final JoystickButton start2 = new JoystickButton(
    driver2,
    XboxController.Button.kStart.value
  );

  private final JoystickButton back2 = new JoystickButton(
    driver2,
    XboxController.Button.kBack.value
  );
  private final LeftTriggerPressed leftTrig = new LeftTriggerPressed(
    driver1,
    1
  );
  private final RightTriggerPressed rightTrig = new RightTriggerPressed();

  final JoystickButton b1 = new JoystickButton(buttonBoard, 1);
  final JoystickButton b2 = new JoystickButton(buttonBoard, 2);
  final JoystickButton b3 = new JoystickButton(buttonBoard, 3);
  final JoystickButton b4 = new JoystickButton(buttonBoard, 4);
  final JoystickButton b5 = new JoystickButton(buttonBoard, 5);
  final JoystickButton b6 = new JoystickButton(buttonBoard, 6);
  Trigger bbStickF = new Trigger(() -> buttonBoard.getRawAxis(1) > 0.7);
  Trigger bbStickB = new Trigger(() -> buttonBoard.getRawAxis(1) < -0.7);

  // private final JoystickAxis rightTrigButton = new JoystickButton(driver1, XboxController.Axis.kRightTrigger)

  /* Subsystems */
  private final Limelight m_Limelight = new Limelight();
  private final Swerve s_Swerve = new Swerve(m_Limelight);
  private final Elevator m_Elevator = new Elevator();
  private final Claw m_Claw = new Claw();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.resetModulesToAbsolute();
    s_Swerve.resetModulesToAbsolute();
    if (Constants.mantis) {
      s_Swerve.setDefaultCommand(
        new TeleopSwerve(
          s_Swerve,
          () -> -driver1.getRawAxis(translationAxis),
          () -> -driver1.getRawAxis(strafeAxis),
          () -> driver1.getRawAxis(rotationAxis),
          () -> back1.getAsBoolean()
        )
      );
    } else {
      s_Swerve.setDefaultCommand(
        new TeleopSwerve(
          s_Swerve,
          () -> -driver1.getRawAxis(translationAxis),
          () -> -driver1.getRawAxis(strafeAxis),
          () -> -driver1.getRawAxis(rotationAxis),
          () -> back1.getAsBoolean()
        )
      );
    }
    m_Elevator.armAndElevator();

    // Configure the button bindings
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
    start1.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    if (!Constants.mantis) {
      //aButton1.onTrue(m_Elevator.armMM1());
      //bButton1.onTrue(m_Elevator.armMM2());
      leftTrig.onTrue(m_Claw.close());
      rightTrig.onTrue(m_Claw.open());
      //rightBumper2.onTrue(m_Elevator.setPosition(150000));
      //leftBumper2.onTrue(m_Elevator.setPosition(10000));

      // aButton1.onTrue(m_Elevator.setPositions(205800, 976));
      // bButton1.onTrue(m_Elevator.setPositions(80000, 1000));
      aButton1.onTrue(m_Claw.motorOn());
      bButton1.onTrue(m_Claw.motorOff());

      yButton1.whileTrue(m_Elevator.armUp());
      xButton1.whileTrue(m_Elevator.armDown());

      start2.onTrue(m_Elevator.resetElevatorEncoder());
      //back2.onTrue(m_Arm.resetArmEncoder());
      leftBumper1.whileTrue(m_Elevator.runDown());
      rightBumper1.whileTrue(m_Elevator.runUp());
      // bButton1.onTrue(s_Swerve.drive1m());
      back1.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }

  public class LeftTriggerPressed extends JoystickButton {

    public LeftTriggerPressed(GenericHID joystick, int buttonNumber) {
      super(joystick, buttonNumber);
      //TODO Auto-generated constructor stub
    }

    @Override
    public boolean getAsBoolean() {
      return driver1.getRawAxis(2) < -0.5;
      // This returns whether the trigger is active
    }
  }

  public class RightTriggerPressed extends Trigger {

    @Override
    public boolean getAsBoolean() {
      return driver1.getRawAxis(2) > 0.5;
      // This returns whether the trigger is active
    }
  }
}
