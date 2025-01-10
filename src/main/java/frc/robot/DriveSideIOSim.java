package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DriveSideIOSim implements DriveSideIO {

    // standard differential drive gearbox reduction
    private static final double kGearboxReduction = 6.86;

    // roughly calculated moment of inertia for a square differential drive, no
    // mechanisms, bumpers; just drivetrain and battery (~45 lbs)
    private static final double kMomentOfInertia = 0.5;
    private static final DCMotor kGearbox = DCMotor.getNEO(2);

    // simulator only needs to simulate a single motor since the gearbox defines 2
    private final DCMotorSim motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kGearbox, kMomentOfInertia, kGearboxReduction), kGearbox);

    private final PIDController pidController = new PIDController(0.3, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

    private boolean driveClosedLoop = false;
    private Voltage driveAppliedVoltage = Volts.of(0);
    private Voltage driveFFVoltage = Volts.of(0);

    private boolean isLeft;

    public DriveSideIOSim(boolean isLeft) {
        this.isLeft = isLeft;
    }

    @Override
    public void updateInputs(DriveSideIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVoltage = driveFFVoltage
                    .plus(Volts.of(pidController.calculate(motor.getAngularVelocity().in(RadiansPerSecond))));
        } else {
            pidController.reset();
        }

        motor.setInputVoltage(MathUtil.clamp(driveAppliedVoltage.in(Volts), -12, 12));

        // You should pass in a real delta time value since this varies slightly
        motor.update(0.02);

        // Update motor inputs
        inputs.connected = true;
        inputs.position = motor.getAngularPosition();
        inputs.velocity = motor.getAngularVelocity();
        inputs.appliedVolts = driveAppliedVoltage;
        inputs.current = Amps.of(motor.getCurrentDrawAmps());

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
        Logger.recordOutput((isLeft ? "Left" : "Right") + " Drive FF V", driveFFVoltage);
        Logger.recordOutput((isLeft ? "Left" : "Right") + " Setpoint", velocity);
        pidController.setSetpoint(radsPerSec);
    }
}
