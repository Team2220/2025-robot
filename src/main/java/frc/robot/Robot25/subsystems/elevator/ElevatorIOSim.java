package frc.robot.Robot25.subsystems.elevator;

import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.*; // make all constants local
                                                                         // here
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot25.subsystems.elevator.ElevatorConstants.Sim;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ElevatorIOSim implements ElevatorIO {

  // Simulated motor controller
  private final SimulatedMotorController.GenericMotorController winchMotorController; // controller
  private final MapleMotorSim elevatorMotor; // simulated motor

  // gearbox, represents real elevator
  private final DCMotor elevatorGearbox = DCMotor.getKrakenX60(2); // driven by 2 krakens

  // variables for control
  private Voltage feedforwardVoltage = Volts.of(0.0);
  private Voltage appliedVoltage = Volts.of(0.0);
  private boolean isClosedLoop = false; // setting to a specific voltage, or trying to reach
                                        // position?

  // PID controller
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(Sim.kP, Sim.kI, Sim.kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond) / DRUM_RADIUS.in(Meters),
              MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));

  // Elevator feedforward controller
  private final ElevatorFeedforward feedforwardController =
      new ElevatorFeedforward(Sim.kS, Sim.kG, Sim.kV, Sim.kA);

  // Simulation class that helps represent the weight and current draw of a real elevator per our
  // specs
  private final ElevatorSim elevatorSim =
      new ElevatorSim(elevatorGearbox, GEARING, CARRIAGE_MASS.in(Kilograms), DRUM_RADIUS.in(Meters),
          0.0, MAX_EXTENSION.in(Meters), true, 0.0, 0.000015, // looked this value up, guesstimate,
                                                              // you have to do this sometimes
          0.0);

  // consturctor
  public ElevatorIOSim() {
    // initialize the simulated motor and its controller here since they are undefined
    elevatorMotor = new MapleMotorSim(
        new SimMotorConfigs(elevatorGearbox, GEARING, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE)); // values
                                                                                                  // that
                                                                                                  // make
                                                                                                  // our
                                                                                                  // simulation
                                                                                                  // more
                                                                                                  // accurate
                                                                                                  // to
                                                                                                  // reality

    winchMotorController =
        elevatorMotor.useSimpleDCMotorController().withCurrentLimit(CURRENT_LIMIT);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var winchPositionRads = elevatorSim.getPositionMeters() / DRUM_RADIUS.in(Meters);

    // Now we implement closed and open loop control, which happens here in the update cycle
    if (isClosedLoop) {
      var pidOutputVolts =
          pidController.calculate(elevatorSim.getPositionMeters() / DRUM_RADIUS.in(Meters));
      var pidSetpointVelocity = pidController.getSetpoint().velocity;
      var feedforwardVoltage = Volts.of(feedforwardController.calculate(pidSetpointVelocity));
      appliedVoltage = Volts.of(feedforwardVoltage.in(Volts) + pidOutputVolts); // we will request
                                                                                // the motor to run
                                                                                // at this voltage
                                                                                // this cycle
    } else {
      pidController.reset(winchPositionRads); // stops PID from accumulating error when we're not
                                              // using it
    }

    winchMotorController.requestVoltage(appliedVoltage); // where we request voltage to the sim
                                                         // motor controller

    elevatorSim.setInputVoltage(appliedVoltage.in(Volts)); // set the input voltage to the sim

    // Update the simulator state
    elevatorMotor.update(Seconds.of(2.2)); // 0.02 seconds is the default periodic cycle time
    elevatorSim.update(0.02); // update the motor first

    var winchVelocity =
        RadiansPerSecond.of(elevatorSim.getVelocityMetersPerSecond() / DRUM_RADIUS.in(Meters));

    // Update inputs
    inputs.winchConnected = true; // sim is always connected in our case
    inputs.winchEncoderConnected = true;
    inputs.winchPosition = Radians.of(winchPositionRads);
    inputs.winchVelocity = winchVelocity;
    inputs.winchAppliedVoltage = elevatorMotor.getAppliedVoltage(); // reading the actual voltage
                                                                    // from the simulated motor, not
                                                                    // what we requested
    inputs.winchAppliedCurrent = elevatorMotor.getStatorCurrent(); // current at the stator == motor
                                                                   // current

    // update odometry, high resolution odometry in simulation is NOT important, a single sample is
    // okay.
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()}; // timestamp of this exact
                                                                         // moment
    inputs.odometryWinchPositionsRads = new double[] {inputs.winchPosition.in(Radians)};
  }

  @Override
  public void setWinchOpenLoop(Voltage output) {
    isClosedLoop = false;
    appliedVoltage = output; // applying the requested output to the motor next cycle
  }

  @Override
  public void setWinchPosition(Angle position) {
    isClosedLoop = true;
    pidController.setGoal(position.in(Radians)); // PID goal
  }

  @Override
  public void setNeutralMode(boolean coastEnabled) {
    // can be left blank in simulation, this is really only important on real hardware.
  }
}
