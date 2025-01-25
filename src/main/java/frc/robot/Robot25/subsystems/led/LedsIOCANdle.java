package frc.robot.Robot25.subsystems.led;

import com.ctre.phoenix.led.CANdle;

public class LedsIOCANdle implements LedsIO {

  private CANdle candle;

  public LedsIOCANdle(int id) {
    candle = new CANdle(id);
  }

  public void updateInputs(LedsIOInputs inputs) {}

  public void setColor(Color color) {
    candle.setLEDs(color.red, color.green, color.blue);
  }
}
