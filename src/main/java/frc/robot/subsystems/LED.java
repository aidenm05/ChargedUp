package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

  private static final CANdle candle1 = new CANdle(Constants.CANdleID1);
  private static final CANdle candle2 = new CANdle(
    Constants.CANdleID2,
    "torch"
  );

  public void CANdleSystem() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.5;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle1.configAllSettings(configAll, 100);
    candle2.configAllSettings(configAll, 100);
  }

  public static void LEDColor(int r, int g, int b) {
    candle1.setLEDs(r, g, b);
    candle2.setLEDs(r, g, b);
  }

  public boolean greenDebounce = true;

  @Override
  public void periodic() {
    if (!DriverStation.getJoystickIsXbox(0)) {
      DriverStation.reportError("Rescan!!!!", false);
      candle1.setLEDs(255, 0, 0);
      candle2.setLEDs(255, 0, 0);
      greenDebounce = false;
    } else {
      if (greenDebounce == false) {
        candle1.setLEDs(0, 255, 0);
        candle2.setLEDs(0, 255, 0);
        greenDebounce = true;
      }
    }
  }
}
