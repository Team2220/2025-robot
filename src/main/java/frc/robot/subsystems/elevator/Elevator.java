package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import frc.lib.devices.TalonFXWrapper;
import static edu.wpi.first.units.Units.*;

public class Elevator {
    TalonFXWrapper elevatorMotorR;
    TalonFXWrapper elevatorMotorL;
    double gearRatio;
    double P;
    double I;
    double D;

    public Elevator() {
        elevatorMotorR = new TalonFXWrapper(
                1,
                "Right Motor",
                true,
                NeutralModeValue.Brake,
                1,
                P,
                I,
                D,
                RotationsPerSecondPerSecond.of(0),
                RotationsPerSecond.of(0),
                false,
                false,
                Rotations.of(0),
                Rotations.of(0),
                null,
                Units.Seconds.of(1),
                Units.Amps.of(75),
                Units.RotationsPerSecond.of(1));

        elevatorMotorL = new TalonFXWrapper(
                2,
                "Left Motor",
                false,
                NeutralModeValue.Brake,
                1,
                P,
                I,
                D,
                RotationsPerSecondPerSecond.of(0),
                RotationsPerSecond.of(0),
                false,
                false,
                Rotations.of(0),
                Rotations.of(0),
                null,
                Units.Seconds.of(1),
                Units.Amps.of(75),
                Units.RotationsPerSecond.of(1));
    }
}
