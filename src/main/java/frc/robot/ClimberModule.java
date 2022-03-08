package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private Solenoid lockerLeftSolenoid;
  private Solenoid lockerRightSolenoid;
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

  private DigitalInput limitLeftSwitch;
  private DigitalInput limitRightSwitch;

  private boolean LOCK = true;
  private boolean UNLOCK = false;
  public ClimberModule(
    int pneumaticsLeftId,
    int pneumaticsRightId,
    int motorLeftId,
    //int motorCenterId,
    int motorRightId,
    boolean inverted,
    int limitSwitchLeftChannel,
    int limitSwitchRightChannel
  ) {
    this.lockerLeftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, pneumaticsLeftId);
    this.lockerRightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, pneumaticsRightId);

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
    
    this.motorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ClimberModuleConstants.kPIDLoopIdx, ClimberModuleConstants.kTimeoutMs);
    //this.motorLeft.setSensorPhase(true);  //NOT NEEDED SINCE ITS INTEGRATED SENSOR

    this.motorLeft.configNominalOutputForward(NOMINAL_FORWARD, ClimberModuleConstants.kTimeoutMs);
		this.motorLeft.configNominalOutputReverse(NOMINAL_REVERSE, ClimberModuleConstants.kTimeoutMs);
		this.motorLeft.configPeakOutputForward(PEAK_FORWARD, ClimberModuleConstants.kTimeoutMs);
		this.motorLeft.configPeakOutputReverse(PEAK_REVERSE, ClimberModuleConstants.kTimeoutMs);
    this.motorLeft.configAllowableClosedloopError(ClimberModuleConstants.kPIDLoopIdx, ALLOWABLE_CLOSELOOP_ERROR, ClimberModuleConstants.kTimeoutMs);
    this.motorLeft.config_kF(ClimberModuleConstants.kPIDLoopIdx, ClimberModuleConstants.kGains.kF, ClimberModuleConstants.kTimeoutMs);
		this.motorLeft.config_kP(ClimberModuleConstants.kPIDLoopIdx, ClimberModuleConstants.kGains.kP, ClimberModuleConstants.kTimeoutMs);
    this.motorLeft.config_kI(ClimberModuleConstants.kPIDLoopIdx, ClimberModuleConstants.kGains.kI, ClimberModuleConstants.kTimeoutMs);
    this.motorLeft.config_kD(ClimberModuleConstants.kPIDLoopIdx, ClimberModuleConstants.kGains.kD, ClimberModuleConstants.kTimeoutMs);

    this.limitLeftSwitch = new DigitalInput(limitSwitchLeftChannel);
    this.limitRightSwitch = new DigitalInput(limitSwitchRightChannel);
  }

  public void setPower(double power) {
    power = power / 2.5;
    this.motorLeft.set(ControlMode.PercentOutput, power);

  }

  public void setSolenoidState(SOLENOID_STATE state, SOLENOID_SIDE side) {
    
    switch (state) {
      case LOCKED:
        if (side == SOLENOID_SIDE.LEFT)
           this.lockerLeftSolenoid.set(LOCK);
        else if (side == SOLENOID_SIDE.RIGHT)
           this.lockerRightSolenoid.set(LOCK);
        else {
           this.lockerLeftSolenoid.set(LOCK);
           this.lockerRightSolenoid.set(LOCK);
        }
        break;
      case UNLOCKED:
        if (side == SOLENOID_SIDE.LEFT)
           this.lockerLeftSolenoid.set(UNLOCK);
        else if (side == SOLENOID_SIDE.RIGHT)
           this.lockerRightSolenoid.set(UNLOCK);
        else {
           this.lockerLeftSolenoid.set(UNLOCK);
           this.lockerRightSolenoid.set(UNLOCK);
        }
        break;
      case UNKNOWN:
        break;
    }
  }

  public boolean contactedLeftPole() {
    return limitLeftSwitch.get();
  }
  public void lockLeftArm() {
    lockerLeftSolenoid.set(true);
  }

  public boolean contactedRightPole() {
    return limitRightSwitch.get();
  }

  public boolean bothArmsMadeContact() {
    return (contactedLeftPole() && contactedRightPole());
  }

  public boolean getLeftSolenoidState() {
    return this.lockerLeftSolenoid.get();
  }

  public boolean getRightSolenoidState() {
    return this.lockerRightSolenoid.get();
  }

  public boolean bothArmLockersClosed() {
    return (this.getLeftSolenoidState() && this.getRightSolenoidState());
  }

  public double getPosition() {
    return this.motorLeft.getSelectedSensorPosition();
  }

  public void setPosition(double pos) {
    this.motorLeft.set(ControlMode.Position, pos);
  }
};
