package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberModule {

  private Solenoid lockerSolenoid;

  // parent
  private TalonFX motorLeft;
  private TalonFX motorRight;

  // private TalonFX motorCenter;

  public enum SOLENOID_STATE {
    LOCKED,
    UNLOCKED,
    UNKNOWN,
  }

  private double RAMP_RATE = 3;

  private DigitalInput limitSwitch;

  private SOLENOID_STATE solenoidState = SOLENOID_STATE.UNKNOWN;

  public ClimberModule(
    int _pheumaticsId,
    int motorLeftId,
    int motorRightId,
    int motorCenterId,
    int limitSwitchChannel
  ) {
    this.lockerSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    this.motorLeft = new TalonFX(motorLeftId);
    this.motorLeft.configOpenloopRamp(RAMP_RATE);
    // this.motorCenter = new TalonFX(motorCenterId);
    this.motorRight.follow(motorLeft);
    // this.motorCenter.follow(motorParentLeft);

    this.limitSwitch = new DigitalInput(limitSwitchChannel);
  }

  public ClimberModule(
    int _pheumaticsId,
    int motorLeftId,
    int motorRightId,
    int motorCenterId,
    boolean inverted,
    int limitSwitchChannel
  ) {
    this.lockerSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    this.motorLeft = new TalonFX(motorLeftId);
    this.motorRight = new TalonFX(motorRightId);
    // this.motorCenter = new TalonFX(motorCenterId);

    this.motorLeft.configOpenloopRamp(RAMP_RATE);

    this.motorLeft.setInverted(inverted);

    this.motorRight.setInverted(inverted);

    // this.motorCenter.setInverted(inverted);

    this.limitSwitch = new DigitalInput(limitSwitchChannel);
  }

  public void setPower(double power) {
    power = power / 2.5;

    this.motorLeft.set(ControlMode.PercentOutput, power);
  }

  public void setSolenoidState(SOLENOID_STATE state) {
    this.solenoidState = state;

    switch (state) {
      case LOCKED:
        this.lockerSolenoid.set(true);
        break;
      case UNLOCKED:
        this.lockerSolenoid.set(false);
        break;
      case UNKNOWN:
        break;
    }
  }

  public boolean contactedPole() {
    return limitSwitch.get();
  }

  public SOLENOID_STATE getSolenoidState() {
    return this.solenoidState;
  }

  /**
   *
   * @return The avarage selected sensor position.
   */
  public double currentPosition() {
    return this.motorLeft.getSelectedSensorPosition();
  }

  public void setPosition(double pos) {
    this.motorLeft.set(ControlMode.Position, pos);
  }
}
