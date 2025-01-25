package frc.robot.Robot25.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedsIO {

  @AutoLog
  public static class LedsIOInputs {
    public boolean isConnected = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LedsIOInputs inputs) {}

  public default void setColor(Color color) {}

  public static class Color {
    int red;
    int blue;
    int green;
    int white;
  }
}
