package frc.robot.Robot25.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.DRUM_RADIUS;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.INITIAL_HEIGHT;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.MAX_EXTENSION;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.MIN_HEIGHT;
import com.ctre.phoenix6.wpiutils.ReplayAutoEnable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput
  public final LoggedMechanism2d mechanism2d =
      new LoggedMechanism2d(3, 3, new Color8Bit(Color.kBlack));

  private final LoggedMechanismRoot2d mechRoot2d = mechanism2d.getRoot("Elevator Root", 1.5, 0);
  private final LoggedMechanismLigament2d elevatorMech2d =
      mechRoot2d.append(new LoggedMechanismLigament2d("Elevator", INITIAL_HEIGHT.in(Meters), 90.0,
          50, new Color8Bit(Color.kBlue)));

  private enum Level {

    minHeight(MIN_HEIGHT), L1(Inches.of(18 + 3)), L2(Inches.of(31.9 + 3)), L3(
        Inches.of(47.6 + 3)), L4(Inches.of(72 + 3));

    private final Distance height;

    Level(Distance height) {
      this.height = height;
    }

    public Distance getHeight() {
      return height;
    }

    public Level up() {
      switch (this) {
        case minHeight:
          return L1;
        case L1:
          return L2;
        case L2:
          return L3;
        case L3:
          return L4;
        default:
          return L4;
      }
    }

    public Level down() {
      switch (this) {
        case L4:
          return L3;
        case L3:
          return L2;
        case L2:
          return L1;
        case L1:
          return minHeight;
        default:
          return minHeight;
      }
    }

  };

  private Level currentLevel;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.minHeight();

    currentLevel = Level.minHeight;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    elevatorMech2d.setLength(radiansToInches(inputs.winchPosition).in(Meters));

    Logger.recordOutput("Elevator/EstimatedHeight",
        radiansToInches(inputs.winchPosition).in(Meters));

    Logger.recordOutput("Elevator/CurrentLevel", currentLevel);

    Logger.recordOutput("Elevator/CurrentLevelHeight", currentLevel.getHeight());
    Logger.recordOutput("Elevator/Radians", inchesToRadians(currentLevel.getHeight()));

    Logger.recordOutput("Elevator/PTuning",
        currentLevel.getHeight().minus(radiansToInches(inputs.winchPosition)));
  }

  private Angle inchesToRadians(Distance d) {
    return Radians.of(d.minus(MIN_HEIGHT).in(Meters) / DRUM_RADIUS.in(Meters));
  }

  private Distance radiansToInches(Angle a) {
    double d = a.in(Radians) * DRUM_RADIUS.in(Meters);
    return Meters.of(d).plus(MIN_HEIGHT);
  }

  public Command minHeight() {
    return this.runOnce(() -> {
      Distance height = Level.minHeight.getHeight();
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
      currentLevel = Level.minHeight;
    });
  }

  public Command L1() {
    return this.runOnce(() -> {
      Distance height = Level.L1.getHeight();
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
      currentLevel = Level.L1;
    });
  }

  public Command L2() {
    return this.runOnce(() -> {
      Distance height = Level.L2.getHeight();
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
      currentLevel = Level.L2;
    });
  }

  public Command L3() {
    return this.runOnce(() -> {
      Distance height = Level.L3.getHeight();
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
      currentLevel = Level.L3;
    });
  }



  public Command L4() {
    return this.runOnce(() -> {
      Distance height = Level.L4.getHeight();
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
      currentLevel = Level.L4;
    });

  }

  public Command maxHeight() {
    return this.runOnce(() -> {
      Distance height = MAX_EXTENSION.plus(MIN_HEIGHT);
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
      currentLevel = Level.L4;
    });
  }

  public Command upLevel() {
    return this.runOnce(() -> {
      currentLevel = currentLevel.up();
      Distance height = currentLevel.getHeight();
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
    });
  }

  public Command downLevel() {
    return this.runOnce(() -> {
      currentLevel = currentLevel.down();
      Distance height = currentLevel.getHeight();
      Angle r = inchesToRadians(height);
      io.setWinchPosition(r);
    });
  }
}
