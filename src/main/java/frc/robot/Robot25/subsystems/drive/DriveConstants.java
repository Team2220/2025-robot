package frc.robot.Robot25.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.units.measure.*;

public class DriveConstants {

  /* These constants only affect real hardware */
  public static class Real {

    /* TODO Both sets of gains need to be tuned to your individual robot */

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs STEER_GAINS =
        new Slot0Configs().withKP(100).withKI(0).withKD(0.5).withKS(0.1).withKV(1.91).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    private static final double METERS_TO_ROTATIONS = 0.3912;
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs().withKP(2.8444 * METERS_TO_ROTATIONS).withKI(0 * METERS_TO_ROTATIONS)
            .withKD(0 * METERS_TO_ROTATIONS).withKS(0.065599).withKV(2.2267 * METERS_TO_ROTATIONS)
            .withKA(0.058183 * METERS_TO_ROTATIONS);
  }

  /* These constants only affect simulation */
  public static class Sim {

    // TODO Both sets of gains need to be tuned to your individual robot; practice
    // tuning in
    // simulation

    /* Steer gains */
    public static final double STEER_KS = 0.1;
    public static final double STEER_KV = 1.91;
    public static final double STEER_KA = 0.0;
    public static final double STEER_KP = 10.0;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.5;

    /* Drive gains */
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 0.124;
    public static final double DRIVE_KA = 0.0;
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    /* These are both calculated off of MK4i Swerve Module CAD */
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.04);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

    private static final Voltage kSteerFrictionVoltage = Volts.of(0.4); // suggested range 0.3 - 0.5
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.7); // suggested range 0.6 - 0.8
  }

  // PathPlanner and Maple Sim config constants
  public static final double ROBOT_MASS_KG = 16.21;
  public static final double ROBOT_MOI = 3.805;
  public static final double WHEEL_COF = 1.542;

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The type of motor used for the drive motor
  private static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the drive motor
  private static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors; we don't use
  // CANcoder so this is
  // irrelevant
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

  // TODO The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final Current kSlipCurrent = Amps.of(120.0);

  // Initial configs for the drive and steer motors and the azimuth encoder; these
  // cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API
  // documentation.
  private static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(90)).withStatorCurrentLimitEnable(true));
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
          // Swerve azimuth does not require much torque output, so we can
          // set a
          // relatively
          // low
          // stator current limit to help avoid brownouts without
          // impacting performance.
          .withStatorCurrentLimit(Amps.of(60)).withStatorCurrentLimitEnable(true));

  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();
  private static final int kPigeonId = 1;

  // CAN bus that the devices are located on;
  // All swerve devices must share the same CAN bus
  public static final CANBus kCANBus = new CANBus();

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 3.8181818181818183;

  private static final double kDriveGearRatio = 6.122; // Source: MK4i swerve module page; L3
                                                       // gearing
  private static final double kSteerGearRatio = 150.0 / 7.0; // Source: MK4i swerve module page
  private static final Distance kWheelRadius = Inches.of(2); 

  // Theoretical free speed (m/s) at 12 V applied output;
  // This needs to be tuned to your individual robot
  public static final double kMaxDriveMotorRPM = 6000.0;
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond
      .of(Math.PI * kWheelRadius.in(Meters) * 2 * kMaxDriveMotorRPM / 60 / kDriveGearRatio); // 5.22
                                                                                             // m/s

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  // Front Left
  private static final int kFrontLeftDriveMotorId = 18;
  private static final int kFrontLeftSteerMotorId = 17;
  private static final int kFrontLeftEncoderId = 1;
  private static final Angle kFrontLeftEncoderOffset = Radians.of(-0.888 + Math.PI);
  private static final boolean kFrontLeftSteerMotorInverted = true;
  private static final boolean kFrontLeftEncoderInverted = false;
  private static final Distance kFrontLeftXPos = Inches.of(10.375);
  private static final Distance kFrontLeftYPos = Inches.of(10.375);

  // Front Right
  private static final int kFrontRightDriveMotorId = 14;
  private static final int kFrontRightSteerMotorId = 13;
  private static final int kFrontRightEncoderId = 2;
  private static final Angle kFrontRightEncoderOffset = Radians.of(-3.738 + Math.PI);
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;
  private static final Distance kFrontRightXPos = Inches.of(10.375);
  private static final Distance kFrontRightYPos = Inches.of(-10.375);

  // Back Left
  private static final int kBackLeftDriveMotorId = 12;
  private static final int kBackLeftSteerMotorId = 11;
  private static final int kBackLeftEncoderId = 0;
  private static final Angle kBackLeftEncoderOffset = Radians.of(-3.276 + Math.PI);
  private static final boolean kBackLeftSteerMotorInverted = true;
  private static final boolean kBackLeftEncoderInverted = false;
  private static final Distance kBackLeftXPos = Inches.of(-10.375);
  private static final Distance kBackLeftYPos = Inches.of(10.375);

  // Back Right
  private static final int kBackRightDriveMotorId = 16;
  private static final int kBackRightSteerMotorId = 15;
  private static final int kBackRightEncoderId = 3;
  private static final Angle kBackRightEncoderOffset = Radians.of(-4.617 + Math.PI);
  private static final boolean kBackRightSteerMotorInverted = true;
  private static final boolean kBackRightEncoderInverted = false;
  private static final Distance kBackRightXPos = Inches.of(-10.375);
  private static final Distance kBackRightYPos = Inches.of(-10.375);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withCANBusName(kCANBus.getName()).withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
      new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
          .withDriveMotorGearRatio(kDriveGearRatio).withSteerMotorGearRatio(kSteerGearRatio)
          .withCouplingGearRatio(kCoupleRatio).withWheelRadius(kWheelRadius)
          .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput).withSlipCurrent(kSlipCurrent)
          .withSpeedAt12Volts(kSpeedAt12Volts).withDriveMotorType(kDriveMotorType)
          .withSteerMotorType(kSteerMotorType).withFeedbackSource(kSteerFeedbackType)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs).withSteerInertia(Sim.kSteerInertia)
          .withDriveInertia(Sim.kDriveInertia).withSteerFrictionVoltage(Sim.kSteerFrictionVoltage)
          .withDriveFrictionVoltage(Sim.kDriveFrictionVoltage).withDriveMotorGains(Real.DRIVE_GAINS)
          .withSteerMotorGains(Real.STEER_GAINS)
          .withEncoderInitialConfigs(new CANcoderConfiguration());

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
      ConstantCreator.createModuleConstants(kFrontLeftSteerMotorId, kFrontLeftDriveMotorId,
          kFrontLeftEncoderId, kFrontLeftEncoderOffset, kFrontLeftXPos, kFrontLeftYPos,
          kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
      ConstantCreator.createModuleConstants(kFrontRightSteerMotorId, kFrontRightDriveMotorId,
          kFrontRightEncoderId, kFrontRightEncoderOffset, kFrontRightXPos, kFrontRightYPos,
          kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
      ConstantCreator.createModuleConstants(kBackLeftSteerMotorId, kBackLeftDriveMotorId,
          kBackLeftEncoderId, kBackLeftEncoderOffset, kBackLeftXPos, kBackLeftYPos, kInvertLeftSide,
          kBackLeftSteerMotorInverted, kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
      ConstantCreator.createModuleConstants(kBackRightSteerMotorId, kBackRightDriveMotorId,
          kBackRightEncoderId, kBackRightEncoderOffset, kBackRightXPos, kBackRightYPos,
          kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted);

  // For max angular velocity calculations
  public static final double DRIVE_BASE_RADIUS = Math.max(
      Math.max(Math.hypot(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontRight.LocationY),
          Math.hypot(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY)),
      Math.max(Math.hypot(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
          Math.hypot(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)));

  public static final double ODOMETRY_FREQUENCY =
      new CANBus(DriveConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
}
