package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

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

  public static final int CLIMBER_MODULE_RATIO = 227;
  public static final int CLIMBER_MODULE_MOTOR_TICK_COUNT = 2049;

  private DoubleSolenoid lockerSolenoid;
  // parent
  private TalonFX motorLeft;
  //private TalonFX motorCenter;
  private TalonFX motorRight;
  private double RAMP_RATE = 3;
  private double NOMINAL_FORWARD = 0.3;
  private double NOMINAL_REVERSE = -0.3;
  private double PEAK_FORWARD = 0.5;
  private double PEAK_REVERSE = -0.5;
  private double ALLOWABLE_CLOSELOOP_ERROR = 400;
  private double MIN_ALLOWABLE_POSITION = 100;
  private double MAX_ALLOWABLE_POSITION =
    0.7 * (ClimberModule.CLIMBER_MODULE_RATIO * 2048);

  private DigitalInput limitLeftSwitch;
  private DigitalInput limitRightSwitch;

  private Value LOCK = Value.kForward;
  private Value UNLOCK = Value.kReverse;

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
        PneumaticsModuleType.REVPH,
        pneumaticsId1,
        pneumaticsId2
      );
    this.motorLeft = new TalonFX(motorLeftId);
    this.motorRight = new TalonFX(motorRightId);
    // this.motorCenter = new TalonFX(motorCenterId);
    this.motorLeft.configFactoryDefault();
    //this.motorCenter.configFactoryDefault();
    this.motorRight.configFactoryDefault();

    //this.motorLeft.configOpenloopRamp(RAMP_RATE);
    this.motorLeft.configClosedloopRamp(RAMP_RATE);
    this.motorLeft.setInverted(inverted);
    this.motorRight.setInverted(inverted);
    // this.motorCenter.setInverted(inverted);
    this.motorRight.follow(motorLeft);
    //this.motorCenter.follow(motorLeft);

    this.motorLeft.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        ClimberModuleConstants.kPIDLoopIdx,
        ClimberModuleConstants.kTimeoutMs
      );
    //this.motorLeft.setSensorPhase(true);  //NOT NEEDED SINCE ITS INTEGRATED SENSOR

    this.motorLeft.configNominalOutputForward(
        NOMINAL_FORWARD,
        ClimberModuleConstants.kTimeoutMs
      );
    this.motorLeft.configNominalOutputReverse(
        NOMINAL_REVERSE,
        ClimberModuleConstants.kTimeoutMs
      );
    this.motorLeft.configPeakOutputForward(
        PEAK_FORWARD,
        ClimberModuleConstants.kTimeoutMs
      );
    this.motorLeft.configPeakOutputReverse(
        PEAK_REVERSE,
        ClimberModuleConstants.kTimeoutMs
      );
    this.motorLeft.configAllowableClosedloopError(
        ClimberModuleConstants.kPIDLoopIdx,
        ALLOWABLE_CLOSELOOP_ERROR,
        ClimberModuleConstants.kTimeoutMs
      );
    this.motorLeft.config_kF(
        ClimberModuleConstants.kPIDLoopIdx,
        ClimberModuleConstants.kGains.kF,
        ClimberModuleConstants.kTimeoutMs
      );
    this.motorLeft.config_kP(
        ClimberModuleConstants.kPIDLoopIdx,
        ClimberModuleConstants.kGains.kP,
        ClimberModuleConstants.kTimeoutMs
      );
    this.motorLeft.config_kI(
        ClimberModuleConstants.kPIDLoopIdx,
        ClimberModuleConstants.kGains.kI,
        ClimberModuleConstants.kTimeoutMs
      );
    this.motorLeft.config_kD(
        ClimberModuleConstants.kPIDLoopIdx,
        ClimberModuleConstants.kGains.kD,
        ClimberModuleConstants.kTimeoutMs
      );

    this.limitLeftSwitch = new DigitalInput(limitSwitchLeftChannel);
    this.limitRightSwitch = new DigitalInput(limitSwitchRightChannel);
  }

  @Deprecated
  public void setPower(double power) {
    power = power / 2.5;
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
    return limitLeftSwitch.get();
  }

  public void lockArm() {
    lockerSolenoid.set(LOCK);
  }

  public boolean contactedRightPole() {
    return limitRightSwitch.get();
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

  public void setPosition(double pos) {
    boolean keepGoing =
      Math.abs(pos) <= MIN_ALLOWABLE_POSITION &&
      Math.abs(pos) >= MAX_ALLOWABLE_POSITION;

    if (keepGoing) {
      this.motorLeft.set(ControlMode.Position, pos);
    }
  }
}
