package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class DriveSideIOSparkMax implements DriveSideIO {

    private static final int kEncoderResolution = 4096;
    private static final int kLeftLeaderChannel = 1;
    private static final int kRightLeaderChannel = 3;
    private static final int kLeftFollowerChannel = 2;
    private static final int kRightFollowerChannel = 4;
    private static final int kLeftEncoderPortA = 0;
    private static final int kLeftEncoderPortB = 1;
    private static final int kRightEncoderPortA = 2;
    private static final int kRightEncoderPortB = 3;

    private final PWMSparkMax leader;
    private final PWMSparkMax follower;

    private final Encoder encoder;

    private final PIDController pidController = new PIDController(1, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

    private final PowerDistribution pdp = new PowerDistribution();

    private final boolean isLeft;

    private boolean driveClosedLoop = false;
    private Voltage driveAppliedVoltage = Volts.of(0);
    private Voltage driveFFVoltage = Volts.of(0);

    public DriveSideIOSparkMax(boolean isLeft) {
        this.isLeft = isLeft;
        leader = new PWMSparkMax(isLeft ? kLeftLeaderChannel : kRightLeaderChannel);
        follower = new PWMSparkMax(isLeft ? kLeftFollowerChannel : kRightFollowerChannel);
        leader.addFollower(follower);
        encoder = new Encoder(isLeft ? kLeftEncoderPortA : kRightEncoderPortA,
                isLeft ? kLeftEncoderPortB : kRightEncoderPortB);

        // Set right side inverted, as in the original implementation
        leader.setInverted(!isLeft);

        // Set the distance per pulse for the drive encoders. We can simply use the
        // angle in radians traveled for one rotation of the wheel divided by the
        // encoder resolution.
        encoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        encoder.reset();
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVoltage = driveFFVoltage.plus(Volts.of(pidController.calculate(encoder.getRate())));
        } else {
            pidController.reset();
        }

        leader.setVoltage(driveAppliedVoltage);

        inputs.connected = true;
        inputs.position = Radians.of(encoder.getDistance());
        inputs.velocity = RadiansPerSecond.of(encoder.getRate());
        inputs.appliedVolts = Volts.of(leader.getVoltage());
        inputs.current = Amps.of(pdp.getCurrent(isLeft ? kLeftLeaderChannel : kRightLeaderChannel));

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestampsSec = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryPositionsRad = new double[] { inputs.position.in(Radians) };
    }

    @Override
    public void setDriveOpenLoop(Voltage output) {
        driveClosedLoop = false;
        driveAppliedVoltage = output;
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        driveClosedLoop = true;
        var radsPerSec = velocity.in(RadiansPerSecond);
        driveFFVoltage = Volts.of(feedforward.calculate(radsPerSec));
        pidController.setSetpoint(radsPerSec);
    }
}