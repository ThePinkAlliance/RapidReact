package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClimberModule {

  public enum SOLENOID_STATE {
    LOCKED,
    UNLOCKED,
    UNKNOWN,
  }

  public enum SOLENOID_SIDE {
    LEFT,
    RIGHT,
    BOTH,
  }
  public static final double CLIMBER_LIMITER = 0.5;
  public static final int SHORT_ARM_MID_CLIMB_START = -355000; //-222222;
  public static final int SHORT_ARM_MID_CLIMB_FINISH = -143195;

  public static final int LONG_ARM_MID_CLIMB_START = -430000; //-222222;
  public static final int LONG_ARM_MID_CLIMB_FINISH = -143195;

  private DoubleSolenoid lockerSolenoid;
  // parent
  private TalonFX motorLeft;
  //private TalonFX motorCenter;
  private TalonFX motorRight;
  private double RAMP_RATE = 0.25;
  private DigitalInput limitLeftSwitch;
  private DigitalInput limitRightSwitch;

  private Value LOCK = Value.kReverse;
  private Value UNLOCK = Value.kForward;
  private Value OFF = Value.kOff;
  private boolean SWITCH_OPEN = true;
  private boolean SWITCH_CLOSE = false;

  public ClimberModule(
    int pneumaticsId1,
    int pneumaticsId2,
    int motorLeftId,
    int motorRightId,
    boolean inverted,
    int limitSwitchLeftChannel,
    int limitSwitchRightChannel
  ) {
    this.lockerSolenoid =
      new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        pneumaticsId1,
        pneumaticsId2
      );
    this.motorLeft = new TalonFX(motorLeftId);
    this.motorRight = new TalonFX(motorRightId);
    // this.motorCenter = new TalonFX(motorCenterId);
    this.motorLeft.configFactoryDefault();
    //this.motorCenter.configFactoryDefault();
    this.motorRight.configFactoryDefault();
    this.motorLeft.setNeutralMode(NeutralMode.Brake);
    this.motorRight.setNeutralMode(NeutralMode.Brake);
    this.motorLeft.configOpenloopRamp(RAMP_RATE);
    this.motorLeft.configClosedloopRamp(RAMP_RATE);
    this.motorLeft.setInverted(inverted);
    this.motorRight.setInverted(inverted);
    // this.motorCenter.setInverted(inverted);
    this.motorRight.follow(motorLeft);
    //this.motorCenter.follow(motorLeft);
    motorLeft.setSelectedSensorPosition(0);

    this.limitLeftSwitch = new DigitalInput(limitSwitchLeftChannel);
    this.limitRightSwitch = new DigitalInput(limitSwitchRightChannel);
  }

  public void moveArms(double power) {
    this.motorLeft.set(ControlMode.PercentOutput, power);
  }

  public void setSolenoidState(SOLENOID_STATE state) {
    switch (state) {
      case LOCKED:
        this.lockerSolenoid.set(LOCK);
        break;
      case UNLOCKED:
        this.lockerSolenoid.set(UNLOCK);
        break;
      case UNKNOWN:
        break;
    }
  }

  public boolean contactedLeftPole() {
    return (limitLeftSwitch.get() == SWITCH_CLOSE);
  }

  public void lockArm() {
    lockerSolenoid.set(LOCK);
  }

  public boolean contactedRightPole() {
    return (limitRightSwitch.get() == SWITCH_CLOSE);
  }

  public boolean bothArmsMadeContact() {
    return (contactedLeftPole() && contactedRightPole());
  }

  public boolean getSolenoidState() {
    return this.lockerSolenoid.get() == LOCK;
  }

  public double getPosition() {
    return this.motorLeft.getSelectedSensorPosition();
  }

  public void resetLeftSensorPosition() {
    this.motorLeft.setSelectedSensorPosition(0);
  }

  public void resetRightSensorPosition() {
    this.motorRight.setSelectedSensorPosition(0);
  }

  public void setPosition(double pos) {
    // boolean keepGoing =
    //   Math.abs(pos) <= MIN_ALLOWABLE_POSITION &&
    //   Math.abs(pos) >= MAX_ALLOWABLE_POSITION;

    //if (keepGoing) {
    this.motorLeft.set(ControlMode.Position, pos);
    //}
  }
}
