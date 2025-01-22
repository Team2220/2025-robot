// Copyright 2021-2025 FRC 6328
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

package frc.robot.Robot25;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Robot25.commands.DriveCommands;
import frc.robot.Robot25.subsystems.drive.Drive;
import frc.robot.Robot25.subsystems.drive.DriveConstants;
import frc.robot.Robot25.subsystems.drive.GyroIO;
import frc.robot.Robot25.subsystems.drive.GyroIONavX;
import frc.robot.Robot25.subsystems.drive.GyroIOSim;
import frc.robot.Robot25.subsystems.drive.ModuleIO;
import frc.robot.Robot25.subsystems.drive.ModuleIOSim;
import frc.robot.Robot25.subsystems.drive.ModuleIOTalonFX;
import frc.robot.SimConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends frc.lib.RobotContainer {
  // Subsystems
  private final Drive drive;

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, SimConstants.SIM_INITIAL_FIELD_POSE);

  // Controller
  private final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandXboxController OperatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    super(driveSimulation);

    // Check for valid swerve config
    var modules = new SwerveModuleConstants[] {DriveConstants.FrontLeft, DriveConstants.FrontRight,
        DriveConstants.BackLeft, DriveConstants.BackRight};
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    switch (SimConstants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new GyroIONavX(), new ModuleIOTalonFX(DriveConstants.FrontLeft),
            new ModuleIOTalonFX(DriveConstants.FrontRight),
            new ModuleIOTalonFX(DriveConstants.BackLeft),
            new ModuleIOTalonFX(DriveConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(driveSimulation.getModules()[0]),
            new ModuleIOSim(driveSimulation.getModules()[1]),
            new ModuleIOSim(driveSimulation.getModules()[2]),
            new ModuleIOSim(driveSimulation.getModules()[3]));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {},
            new ModuleIO() {});
        break;
    }



    NamedCommands.registerCommand("Test", drive.NullCommand(1));
    NamedCommands.registerCommand("ToggleCoast", drive.toggleCoastCommand());
    NamedCommands.registerCommand("Null", drive.NullCommand(2));



    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Static Drive Voltage", Commands.run(() -> drive.driveOpenLoop(10)));
    autoChooser.addOption("Static Turn Voltage", Commands.run(() -> drive.TurnOpenLoop(10)));

    // Set up SysId routines
    autoChooser.addOption("Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption("Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Xbox controller is mapped incorrectly on Mac OS
    DoubleSupplier xSupplier = () -> !SimConstants.IS_MAC ? DriverController.getRightX()
        : DriverController.getRightTriggerAxis();
    DoubleSupplier ySupplier = () -> !SimConstants.IS_MAC ? -DriverController.getRightY()
        : -DriverController.getLeftTriggerAxis();
    DoubleSupplier omegaSupplier = () -> -DriverController.getLeftX();
    BooleanSupplier slowModeSupplier =
        () -> !SimConstants.IS_MAC ? DriverController.getRightTriggerAxis() > 0.5
            : DriverController.getRightX() > 0.0;

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, slowModeSupplier));

    DriverController.a()
        .toggleOnTrue(DriveCommands.keepRotationForward(drive, xSupplier, ySupplier));

    // POV snap to angles
    DriverController.povUp().onTrue(DriveCommands.snapToRotation(drive, Rotation2d.kZero));
    DriverController.povUpRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-45)));
    DriverController.povRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-90)));
    DriverController.povDownRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-135)));
    DriverController.povDown()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-180)));
    DriverController.povDownLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(135)));
    DriverController.povLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(90)));
    DriverController.povUpLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(45)));

    // TODO should this be true in TeleOp?
    // Switch to X pattern when X button is pressed
    DriverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when START button is pressed
    DriverController.start()
        .onTrue(Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
            drive).ignoringDisable(true));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public Command getTestCommand() {
    return autoChooser.get();
  }

  @Override
  public void disabledInit() {
    drive.stopWithX();
  }
}
