package frc.robot.Robot25.subsystems.outtake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.devices.TalonFXWrapper;
import frc.lib.devices.TalonFXWrapper.FollowerConfig;
import frc.robot.Robot25.subsystems.outtake.OuttakeIO;
import frc.robot.Robot25.subsystems.outtake.OuttakeIO.OuttakeIOInputs;

public class OuttakeIOTalonFX implements OuttakeIO {
  TalonFXWrapper outtakeTalonFX;
  double gearRatio;
  double P;
  double I;
  double D;
  final int rightMotorID = 1;
  final int leftMotorID = 2;

  public OuttakeIOTalonFX() {
        outtakeTalonFX = new TalonFXWrapper(
                rightMotorID,
                "Outtake",
                true,
                NeutralModeValue.Brake,
                gearRatio,
                P,
                I,
                D,
                RotationsPerSecondPerSecond.of(0),
                RotationsPerSecond.of(0),
                false,
                false,
                Rotations.of(120.0 / 360.0),
                Rotations.of(0),
                new FollowerConfig(leftMotorID, true),
                Units.Seconds.of(3),
                Units.Amps.of(75),
                Units.RotationsPerSecond.of(1));
    }

  @Override
  public void setOpenLoop(Voltage output) {
    outtakeTalonFX.setVoltageOut(output);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.winchConnected = false;
    inputs.winchVelocity = outtakeTalonFX.getVelocity();
    inputs.winchPosition = outtakeTalonFX.getPosition();
    inputs.winchCurrent = outtakeTalonFX.getTorqueCurrent();
    inputs.winchAppliedVolts = Volts.of(0);
  }
}
