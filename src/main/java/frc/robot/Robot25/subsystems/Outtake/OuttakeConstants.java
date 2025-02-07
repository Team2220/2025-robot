package frc.robot.Robot25.subsystems.outtake;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

public class OuttakeConstants {
  public static final double GEARING = 5.0;
  public static final Current CURRENT_LIMIT = Amps.of(40);

  public static final class Sim {
    public static final double kP = 4.0; // 5
    public static final double kI = 0.3;
    public static final double kD = 0.6;
    public static final double kS = 0.0;
    public static final double kG = 0.43; // 0.37
    public static final double kV = 0.10146; // 2.67
    public static final double kA = 0.002; // * DRUM_RADIUS.in(Meters); // 0.05
    public static final MomentOfInertia MOTOR_LOAD_MOI = KilogramSquareMeters.of(0.04); // TODO estimate
    public static final Voltage FRICTION_VOLTAGE = Volts.of(0.5);
  }
}
