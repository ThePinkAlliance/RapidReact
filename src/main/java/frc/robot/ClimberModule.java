package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberModule extends SubsystemBase {

  private final int PRIMARY_PID_SLOT = 0;
  private final int TIMEOUT_MS = 100;

  private Solenoid lockerSolenoid;
  private TalonFX motorLeft;
  private TalonFX motorRight;
  private TalonFX motorCenter;
  private PIDController angleController;

  public enum SOLENOID_STATE {
    LOCKED,
    UNLOCKED,
    UNKNOWN,
  }

  private DigitalInput limitSwitch;

  private SOLENOID_STATE solenoidState = SOLENOID_STATE.UNKNOWN;
  private double targetPosition = 0;

  private final double kP = 0.02;
  private final double kI = 0.002;

  public ClimberModule(
    int _pheumaticsId,
    int motorLeftId,
    int motorRightId,
    int motorCenterId,
    int limitSwitchChannel
  ) {
    this.lockerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    this.motorLeft = new TalonFX(motorLeftId);
    this.motorRight = new TalonFX(motorRightId);
    this.motorCenter = new TalonFX(motorCenterId);

    this.angleController = new PIDController(0, 0, 0);

    this.motorLeft.config_kP(PRIMARY_PID_SLOT, kP, TIMEOUT_MS);
    this.motorLeft.config_kI(PRIMARY_PID_SLOT, kI, TIMEOUT_MS);

    this.motorRight.config_kP(PRIMARY_PID_SLOT, kP, TIMEOUT_MS);
    this.motorRight.config_kI(PRIMARY_PID_SLOT, kI, TIMEOUT_MS);

    this.motorCenter.config_kP(PRIMARY_PID_SLOT, kP, TIMEOUT_MS);
    this.motorCenter.config_kI(PRIMARY_PID_SLOT, kI, TIMEOUT_MS);

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
    this.lockerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    this.motorLeft = new TalonFX(motorLeftId);
    this.motorRight = new TalonFX(motorRightId);
    this.motorCenter = new TalonFX(motorCenterId);

    this.angleController = new PIDController(0, 0, 0);

    this.motorLeft.config_kP(PRIMARY_PID_SLOT, kP, TIMEOUT_MS);
    this.motorLeft.config_kI(PRIMARY_PID_SLOT, kI, TIMEOUT_MS);

    this.motorRight.config_kP(PRIMARY_PID_SLOT, kP, TIMEOUT_MS);
    this.motorRight.config_kI(PRIMARY_PID_SLOT, kI, TIMEOUT_MS);

    this.motorCenter.config_kP(PRIMARY_PID_SLOT, kP, TIMEOUT_MS);
    this.motorCenter.config_kI(PRIMARY_PID_SLOT, kI, TIMEOUT_MS);

    this.motorLeft.setInverted(inverted);
    this.motorLeft.setInverted(inverted);

    this.motorRight.setInverted(inverted);
    this.motorRight.setInverted(inverted);

    this.motorCenter.setInverted(inverted);
    this.motorCenter.setInverted(inverted);

    this.limitSwitch = new DigitalInput(limitSwitchChannel);
  }

  public void setPower(double power) {
    this.motorLeft.set(ControlMode.PercentOutput, power);
    this.motorRight.set(ControlMode.PercentOutput, power);
    this.motorCenter.set(ControlMode.PercentOutput, power);
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
    double l = motorLeft.getSelectedSensorPosition();
    double r = motorRight.getSelectedSensorPosition();
    double c = motorCenter.getSelectedSensorPosition();

    return (l + r + c) / 3.0;
  }

  public void setPosition(double pos) {
    targetPosition = pos;

    this.motorLeft.set(ControlMode.Position, pos);
    this.motorRight.set(ControlMode.Position, pos);
    this.motorCenter.set(ControlMode.Position, pos);
  }
}
