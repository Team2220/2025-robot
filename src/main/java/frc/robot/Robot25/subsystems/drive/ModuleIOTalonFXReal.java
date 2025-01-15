// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Robot25.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.devices.PWMEncoder;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXReal extends ModuleIOTalonFX {

  // Queue to read inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // We use an analog absolute encoder
  private final PWMEncoder absoluteEncoder;

  public ModuleIOTalonFXReal(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Pass constants to parent class with updated gains
    super(
        constants
            .withDriveMotorGains(DriveConstants.Real.DRIVE_GRAINS)
            .withSteerMotorGains(DriveConstants.Real.STEER_GAINS));

    // TODO should this be Rotations.of or Radians.of?
    absoluteEncoder = new PWMEncoder(constants.EncoderId, Rotations.of(constants.EncoderOffset));

    this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    this.drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
    // I believe this should be supplied in rotations
    this.turnPositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(() -> absoluteEncoder.getPosition().in(Rotations));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update turn position
    inputs.turnAbsolutePosition = Rotation2d.fromRadians(absoluteEncoder.getPosition().in(Radians));
    // TODO analog encoder doesn't provide a connected status, could measure voltage above a
    // threshold?
    inputs.turnEncoderConnected = true;

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }
}
