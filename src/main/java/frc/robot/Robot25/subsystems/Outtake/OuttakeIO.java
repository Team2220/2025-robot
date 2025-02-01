package frc.robot.Robot25.subsystems.outtake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot25.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    public boolean winchConnected = false;
    public Angle winchPosition = Radians.of(0.0);
    public AngularVelocity winchVelocity = RadiansPerSecond.of(0.0);
    public Voltage winchAppliedVolts = Volts.of(0.0);
    public Current winchCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(OuttakeIOInputs inputs) {}

  /** Run the drive side at the specified open loop value. */
  public default void setOpenLoop(Voltage output) {}

}
