package frc.robot.Robot25.subsystems.led;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot25.subsystems.led.LedsIO.Color;
import org.littletonrobotics.junction.Logger;

public class Leds {
  private final LedsIO io;
  private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

  private final Alert ledsDisconnectedAlert;

  public Leds(LedsIO io) {
    this.io = io;
    ledsDisconnectedAlert = new Alert("Disconnected leds.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Leds", inputs);

    // Update alerts
    ledsDisconnectedAlert.set(!inputs.isConnected);
  }

  public void setColor(Color color) {
    io.setColor(color);
  }

}
