package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final Joystick driver1 = new Joystick(0);
  // private final Joystick driver2 = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
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
  // private final JoystickButton aButton2 = new JoystickButton(
  //   driver2,
  //   XboxController.Button.kA.value
  // );
  // private final JoystickButton bButton2 = new JoystickButton(
  //   driver2,
  //   XboxController.Button.kB.value
  // );
  // private final JoystickButton xButton2 = new JoystickButton(
  //   driver2,
  //   XboxController.Button.kX.value
  // );
  // private final JoystickButton yButton2 = new JoystickButton(
  //   driver2,
  //   XboxController.Button.kY.value
  // );

  /* Subsystems */
  private final Limelight m_Limelight = new Limelight();
  private final Swerve s_Swerve = new Swerve(m_Limelight);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -driver1.getRawAxis(translationAxis),
        () -> -driver1.getRawAxis(strafeAxis),
        () -> -driver1.getRawAxis(rotationAxis),
        () -> leftBumper1.getAsBoolean()
      )
    );

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
    yButton1.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    //runUp.whileTrue(new RunCommand(() -> m_Elevator.increasePosition()));
    //runDown.whileTrue(new RunCommand(() -> m_Elevator.decreasePosition()));

    if (Constants.mantis == false) {
      final Elevator m_Elevator = new Elevator();
      aButton1.whileTrue(m_Elevator.runUp());
      bButton1.whileTrue(m_Elevator.runDown());
    }

    // aButton1.onTrue(new InstantCommand(() -> m_Elevator.runUp()));
    // aButton1.onFalse(new InstantCommand(() -> m_Elevator.stop()));
    // bButton1.onTrue(new InstantCommand(() -> m_Elevator.runDown()));
    // bButton1.onFalse(new InstantCommand(() -> m_Elevator.stop()));
    // rightBumper1.onTrue(new InstantCommand(() -> m_Elevator.reset()));
    xButton1.onTrue(s_Swerve.drive1m());
    // moveToGoalAprilTags.onTrue(s_Swerve.moveToGoalAprilTags());
    // moveToGoalRetro.onTrue(s_Swerve.moveToGoalRetroreflective());
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
}
