// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a differential drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters

  private static final Pose2d kInitialPose = new Pose2d(2, 6, Rotation2d.fromDegrees(0));

  // IO interfaces
  private final DriveSideIO leftDriveIO;
  private final DriveSideIO rightDriveIO;
  private final GyroIO gyroIO;

  // IO inputs
  private DriveSideIOInputsAutoLogged leftDriveIOInputs = new DriveSideIOInputsAutoLogged();
  private DriveSideIOInputsAutoLogged rightDriveIOInputs = new DriveSideIOInputsAutoLogged();
  private GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
      m_kinematics, Rotation2d.kZero, 0, 0, kInitialPose);

  private Rotation2d rawGyroRotation = Rotation2d.kZero;

  private DifferentialDriveWheelPositions lastWheelPositions = new DifferentialDriveWheelPositions(0, 0);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse
   * and resets the
   * gyro.
   */
  public Drivetrain(DriveSideIO left, DriveSideIO right, GyroIO gyro) {
    leftDriveIO = left;
    rightDriveIO = right;
    gyroIO = gyro;

    gyroIO.reset();

    leftDriveIO.updateInputs(leftDriveIOInputs);
    rightDriveIO.updateInputs(rightDriveIOInputs);
    gyroIO.updateInputs(gyroIOInputs);

    var wheelPositions = getDifferentialDrivePositions();

    poseEstimator.updateWithTime(0, rawGyroRotation, wheelPositions.leftMeters, wheelPositions.rightMeters);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftAngularVelocity = RadiansPerSecond.of(speeds.leftMetersPerSecond / kWheelRadius);
    var rightAngularVelocity = RadiansPerSecond.of(speeds.rightMetersPerSecond / kWheelRadius);
    leftDriveIO.setDriveVelocity(leftAngularVelocity);
    rightDriveIO.setDriveVelocity(rightAngularVelocity);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public DifferentialDriveWheelPositions getDifferentialDrivePositions() {
    var leftPositionMeters = leftDriveIOInputs.position.in(Radians) * kWheelRadius;
    var rightPositionMeters = rightDriveIOInputs.position.in(Radians) * kWheelRadius;
    return new DifferentialDriveWheelPositions(leftPositionMeters, rightPositionMeters);
  }

  @Override
  public void periodic() {
    // Update the hardware IO inputs every period
    leftDriveIO.updateInputs(leftDriveIOInputs);
    rightDriveIO.updateInputs(rightDriveIOInputs);
    gyroIO.updateInputs(gyroIOInputs);

    // This logs the inputs during REAL or SIM mode, or replays them if in REPLAY
    // mode
    Logger.processInputs("Drivetrain/LeftDrive", leftDriveIOInputs);
    Logger.processInputs("Drivetrain/RightDrive", rightDriveIOInputs);
    Logger.processInputs("Drivetrain/Gyro", gyroIOInputs);

    // Stop the robot if disabled
    if (DriverStation.isDisabled()) {
      leftDriveIO.setDriveOpenLoop(Volts.of(0));
      rightDriveIO.setDriveOpenLoop(Volts.of(0));
    }

    // Update odometry, all signals are sampled together
    var sampleTimestamps = leftDriveIOInputs.odometryTimestampsSec;
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // The simulator abstraction for the gyro is tightly coupled to the Drivetrain
      // subsytem, so it cannot be implemented in its own class, it must be here
      var wheelPositions = getDifferentialDrivePositions();
      if (gyroIOInputs.connected) {
        // Real gyro rotation
        rawGyroRotation = gyroIOInputs.yawPosition;
      } else {
        // Simulated gyro rotation
        rawGyroRotation = rawGyroRotation
            .plus(Rotation2d.fromRadians(m_kinematics.toTwist2d(lastWheelPositions, wheelPositions).dtheta));
      }

      // update last wheel positions for delta
      lastWheelPositions = wheelPositions;

      // update pose estimator
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, lastWheelPositions.leftMeters,
          lastWheelPositions.rightMeters);
    }

    // Log the robot pose
    Logger.recordOutput("Robot Pose", poseEstimator.getEstimatedPosition());
  }
}
