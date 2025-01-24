package frc.robot.Robot25.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.DRUM_RADIUS;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.INITIAL_HEIGHT;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.MAX_EXTENSION;
import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.MIN_HEIGHT;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {

  // The hardware IO implementation this subsystem is using
  private final ElevatorIO io;

  // inputs autologged, this class gets created automatically when you use the @AutoLog annotation
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // We're going to make a mechanism object so we can "see" what the elevator is doing
  @AutoLogOutput
  public final LoggedMechanism2d mech2d =
      new LoggedMechanism2d(100, 100, new Color8Bit(Color.kBlack)); // this too

  private final LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 50, 0); // just
                                                                                           // copy
                                                                                           // this
                                                                                           // mech2d
                                                                                           // stuff
  private final LoggedMechanismLigament2d elevatorMech2d =
      mech2dRoot.append(new LoggedMechanismLigament2d("Elevator", INITIAL_HEIGHT.in(Inches), 90.0,
          50.0, new Color8Bit(Color.kFirstRed))); // oops :)

  private Angle elevatorPosition = Radians.of(0.0);

  public Elevator(ElevatorIO io) {
    this.io = io; // assigns the class member 'io' to the given parameter 'io'

    elevatorPosition = fromElevatorHeight(MIN_HEIGHT); // set the initial height when constructed
  }

  @Override
  public void periodic() {
    // important part: we need to call the updateInputs method on the hardware IO implementation
    // given every periodic cycle
    io.updateInputs(inputs); // updates our 'inputs' member

    // this part is what allows replay:
    Logger.processInputs("Elevator", inputs); // takes the inputs from the log file and plays them
                                              // in the subsystem, mimicking a real implementation

    io.setWinchPosition(elevatorPosition);
    elevatorMech2d.setLength(fromAngular(inputs.winchPosition).in(Inches)); // doing height in
                                                                            // inches
  }

  /* These I already calculated, just copy them */

  public void moveToMinimum() {
    elevatorPosition = fromElevatorHeight(MIN_HEIGHT);
  }

  public void moveToL1() {
    elevatorPosition = fromElevatorHeight(Inches.of(21));
  }

  public void moveToL2() {
    elevatorPosition = fromElevatorHeight(Inches.of(34.9));
  }

  public void moveToL3() {
    elevatorPosition = fromElevatorHeight(Inches.of(50.6));
  }

  public void moveToL4() {
    elevatorPosition = fromElevatorHeight(Inches.of(75));
  }

  public void moveToMaximum() {
    elevatorPosition = fromElevatorHeight(MIN_HEIGHT.plus(MAX_EXTENSION));
  }

  /*
   * ### We're going make two helper functions, angle position is measured in radians because they
   * are rotations, and the elevator measures its position in meters since it's a height ###
   */

  static Distance fromAngular(Angle winchPosition) {
    return Meters.of(winchPosition.in(Radians) * DRUM_RADIUS.in(Meters)).plus(MIN_HEIGHT);
  }

  static Angle fromElevatorHeight(Distance elevatorHeight) {
    return Radians.of(elevatorHeight.minus(MIN_HEIGHT).in(Meters) / DRUM_RADIUS.in(Meters));
  }
}
