package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public AngularVelocity yawVelocity = RadiansPerSecond.of(0.0);

        public double[] odometryTimestampsSec = new double[] {};
        public double[] odometryYawPositionsRad = new double[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default void reset() {
    }
}
