package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOAnalog implements GyroIO {

    private final AnalogGyro gyro = new AnalogGyro(0);
    private final AnalogInput input = new AnalogInput(0);

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // A disconnected analog device typically reads very close to 0V, no other
        // method for this
        inputs.connected = input.getVoltage() > 0.1;
        inputs.yawPosition = gyro.getRotation2d();
        inputs.yawVelocity = DegreesPerSecond.of(gyro.getRate());

        // Analog gyros don't do timestamped odometry with queues
        inputs.odometryTimestampsSec = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryYawPositionsRad = new double[] { inputs.yawPosition.getRadians() };
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
