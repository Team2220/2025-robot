package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface DriveSideIO {

    @AutoLog
    public static class DriveSideIOInputs {
        public boolean connected = false;
        public Angle position = Radians.of(0.0);
        public AngularVelocity velocity = RadiansPerSecond.of(0.0);
        public Voltage appliedVolts = Volts.of(0.0);
        public Current current = Amps.of(0.0);

        public double[] odometryTimestampsSec = new double[] {};
        public double[] odometryPositionsRad = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DriveSideIOInputs inputs) {
    }

    /** Run the drive side at the specified open loop value. */
    public default void setDriveOpenLoop(Voltage output) {
    }

    /** Run the drive side at the specified velocity. */
    public default void setDriveVelocity(AngularVelocity velocity) {
    }
}
