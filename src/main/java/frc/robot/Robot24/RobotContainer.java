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

package frc.robot.Robot24;

import static frc.robot.Constants.*;

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
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.Robot24.commands.DriveCommands;
import frc.robot.Robot24.generated.TunerConstants;
import frc.robot.Robot24.subsystems.drive.Drive;
import frc.robot.Robot24.subsystems.drive.GyroIO;
import frc.robot.Robot24.subsystems.drive.GyroIONavX;
import frc.robot.Robot24.subsystems.drive.GyroIOSim;
import frc.robot.Robot24.subsystems.drive.ModuleIO;
import frc.robot.Robot24.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.Robot24.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.Robot24.subsystems.elevator.Elevator;
import frc.robot.Robot24.subsystems.elevator.ElevatorIO;
import frc.robot.Robot24.subsystems.elevator.ElevatorIOSim;
import frc.robot.Robot24.subsystems.elevator.ElevatorIOTalonFX;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends frc.lib.RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(Drive.mapleSimConfig, SIM_INITIAL_FIELD_POSE);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController elevatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    super(driveSimulation);

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // TODO hardware abstraction
    switch (CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
        
        elevator = new Elevator(new ElevatorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            // Xbox controller is mapped incorrectly on Mac OS
            () ->
                SIM_MODE == Mode.REAL ? -controller.getRightX() : -controller.getLeftTriggerAxis(),
            () ->
                SIM_MODE == Mode.REAL
                    ? controller.getRightTriggerAxis() > 0.5
                    : controller.getRightY() > 0.5));

    controller
        .a()
        .toggleOnTrue(
            DriveCommands.keepRotationForward(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    controller.povUp().onTrue(DriveCommands.snapToRotation(drive, Rotation2d.kZero));

    controller
        .povUpRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-45)));

    controller.povRight().onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-90)));

    controller
        .povDownRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-135)));

    controller.povDown().onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-180)));

    controller
        .povDownLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(135)));

    controller.povLeft().onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(90)));

    controller.povUpLeft().onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(45)));

    // Lock to 0° when A button is held
    // controller
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    elevatorController.povDown().onTrue(Commands.runOnce(
        () -> {
            elevator.minHeight();
        }
    ));
    elevatorController.povUp().onTrue(Commands.runOnce(
        () -> {
            elevator.maxHeight();
        }
    ));
    elevatorController.a().onTrue(Commands.runOnce(
        () -> {
            elevator.L1();
        }
    ));
    elevatorController.x().onTrue(Commands.runOnce(
        () -> {
            elevator.L2();
        }
    ));
    elevatorController.b().onTrue(Commands.runOnce(
        () -> {
            elevator.L3();
        }
    ));
    elevatorController.y().onTrue(Commands.runOnce(
        () -> {
            elevator.L4();
        }
    ));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public Command getTestCommand() {
    return autoChooser.get();
  }
}
