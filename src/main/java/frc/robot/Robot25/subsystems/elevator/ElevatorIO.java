package frc.robot.Robot25.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean winchConnected = false;
    public boolean winchEncoderConnected = false;
    public Angle winchPosition = Radians.of(0.0);
    public AngularVelocity winchVelocity = RadiansPerSecond.of(0.0);
    public Voltage winchAppliedVoltage = Volts.of(0.0);
    public Current winchAppliedCurrent = Amps.of(0.0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryWinchPositionsRads = new double[] {};
  }

  /** Updates the set of loggable inputs each periodic cycle */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the winch motor at the specified open loop voltage */
  public default void setWinchOpenLoop(Voltage output) {}

  /** Run the winch motor to the specified position */
  public default void setWinchPosition(Angle position) {}

  /** Set the neutral mode of the winch motor */
  public default void setNeutralMode(boolean coastEnabled) {}
}
