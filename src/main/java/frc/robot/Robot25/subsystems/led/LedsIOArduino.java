package frc.robot.Robot25.subsystems.led;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix.led.CANdle;
import com.google.errorprone.annotations.CanIgnoreReturnValue;
import frc.robot.Robot25.subsystems.led.Arduino.Arduino;
import frc.robot.Robot25.subsystems.led.LedsIO.Color;
import frc.robot.Robot25.subsystems.led.LedsIO.LedsIOInputs;
import java.net.CacheRequest;
import java.net.CacheResponse;
import java.util.Calendar;

public class LedsIOArduino implements LedsIO {

  private Arduino arduino;

  public LedsIOArduino() {
    arduino = new Arduino();
  }

  public void updateInputs(LedsIOInputs inputs) {
    inputs.isConnected = arduino.isConnected();
  }

  public void setColor(Color color) {
    arduino.customColor(color.red, color.green, color.blue);
  }
}
