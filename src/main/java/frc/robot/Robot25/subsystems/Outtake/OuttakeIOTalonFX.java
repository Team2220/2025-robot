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
  final int motorID = 25;

  public OuttakeIOTalonFX() {
    outtakeTalonFX = new TalonFXWrapper(
        motorID,
        "Outtake",
        true,
        NeutralModeValue.Brake);
  }

  @Override
  public void setOpenLoop(Voltage output) {
    outtakeTalonFX.setVoltageOut(output);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.outtakeConnected = false;
    inputs.outtakeVelocity = outtakeTalonFX.getVelocity();
    inputs.outtakeCurrent = outtakeTalonFX.getTorqueCurrent();
    inputs.outtakeAppliedVolts = Volts.of(0);
  }
}
