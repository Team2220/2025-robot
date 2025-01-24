package frc.robot.Robot25.subsystems.elevator;

import static frc.robot.Robot25.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.Robot25.util.PhoenixUtil.tryUntilOk;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot24.subsystems.drive.PhoenixOdometryThread;
import java.util.Queue;

public class ElevatorIOTalonFX implements ElevatorIO {

  // Real motor controllers; this implementation is about using TalonFX's, like our real robot.
  private final TalonFX leadWinchMotor = new TalonFX(Real.LEAD_WINCH_MOTOR_ID);
  private final TalonFX followerWinchMotor = new TalonFX(Real.FOLLOWER_WINCH_MOTOR_ID); // 2 talons,
                                                                                        // the other
                                                                                        // will
                                                                                        // mirror
                                                                                        // the first

  // Requests for the motor controller. Specific to Talons and kind of high level, just copy
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);

  // Timstamps from the Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from the winch motor
  private final StatusSignal<Angle> winchPosition;
  private final Queue<Double> winchPositionQueue;
  private final StatusSignal<AngularVelocity> winchVelocity;
  private final StatusSignal<Voltage> winchAppliedVoltage;
  private final StatusSignal<Current> winchAppliedCurrent;

  // Connection debouncer (i'll explain)
  private final Debouncer winchConnectedDebouncer = new Debouncer(0.5);

  public ElevatorIOTalonFX() {
    var winchMotorConfig = new TalonFXConfiguration();
    winchMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // brake when uncontrolled
    winchMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // we'll have to know
                                                                              // the real motor
                                                                              // mount to determine
                                                                              // this
    // PID gains, just like how we set these for a PID controller, except we use the onboard TalonFX
    // pid controller in this case
    var winchGains = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(Real.kG).withKS(Real.kS).withKV(Real.kV).withKA(Real.kA).withKP(Real.kP)
        .withKI(Real.kI).withKD(Real.kD);

    // need to actually assign the gains to the config
    winchMotorConfig.Slot0 = winchGains;

    // use try until okay since electronics don't always play nicely with code.
    tryUntilOk(5, () -> leadWinchMotor.getConfigurator().apply(winchMotorConfig, 0.25));
    tryUntilOk(5, () -> leadWinchMotor.setPosition(0.0, 0.25)); // reset the encoder
    tryUntilOk(5,
        () -> followerWinchMotor.setControl(new Follower(Real.LEAD_WINCH_MOTOR_ID, true))); // follow
                                                                                            // the
                                                                                            // lead
                                                                                            // winch
                                                                                            // motor,
                                                                                            // invert
                                                                                            // its
                                                                                            // output
                                                                                            // since
                                                                                            // its
                                                                                            // on
                                                                                            // the
                                                                                            // opposite
                                                                                            // side

    // Create the timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    winchPosition = leadWinchMotor.getPosition();
    winchPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(leadWinchMotor.getPosition());
    winchVelocity = leadWinchMotor.getVelocity();
    winchAppliedVoltage = leadWinchMotor.getMotorVoltage();
    winchAppliedCurrent = leadWinchMotor.getStatorCurrent();

    // Configure periodic frames to update these values
    BaseStatusSignal.setUpdateFrequencyForAll(Real.ODOMETRY_FREQUENCY, winchPosition); // update
                                                                                       // these at
                                                                                       // this rate
                                                                                       // since they
                                                                                       // are tied
                                                                                       // to
                                                                                       // odometry
                                                                                       // calculations
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, winchVelocity, winchAppliedVoltage,
        winchAppliedCurrent); // update these at a different rate
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Refresh all signals every periodic cycle
    var winchStatus = BaseStatusSignal.refreshAll(winchPosition, winchVelocity, winchAppliedVoltage,
        winchAppliedCurrent);

    // Update winch inputs
    inputs.winchConnected = winchConnectedDebouncer.calculate(winchStatus.isOK());
    inputs.winchEncoderConnected = false; // we aren't using an external encoder in this hardware
                                          // implementation
    inputs.winchPosition = winchPosition.getValue();
    inputs.winchVelocity = winchVelocity.getValue();
    inputs.winchAppliedVoltage = winchAppliedVoltage.getValue();
    inputs.winchAppliedCurrent = winchAppliedCurrent.getValue();
  }

  @Override
  public void setWinchOpenLoop(Voltage output) {
    leadWinchMotor.setControl(voltageRequest.withOutput(output));
    // remember, we configured the follower motor to mirror what the lead does. we only measure and
    // control the lead
  }

  @Override
  public void setWinchPosition(Angle position) {
    leadWinchMotor.setControl(positionVoltageRequest.withPosition(position));
  }

  @Override
  public void setNeutralMode(boolean coastEnabled) {
    var neutralMode = coastEnabled ? NeutralModeValue.Coast : NeutralModeValue.Brake; // coast when
                                                                                      // input is
                                                                                      // true, brake
                                                                                      // when input
                                                                                      // is false.
    leadWinchMotor.setNeutralMode(neutralMode);
    followerWinchMotor.setNeutralMode(neutralMode); // not sure if setting talon to follow mirrors
                                                    // the neutral mode, so also set it here.
  }
}
