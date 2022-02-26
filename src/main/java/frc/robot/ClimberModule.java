package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberModule {

  private final int PRIMARY_PID_SLOT = 0;
  private final int TIMEOUT_MS = 100;

  private Solenoid lockerSolenoid;
  private TalonFX angleMotor;
  private PIDController angleController;

  public enum SOLENOID_STATE {
    LOCKED,
    UNLOCKED,
    UNKNOWN,
  }

  private DigitalInput limitSwitch;

  private SOLENOID_STATE solenoidState = SOLENOID_STATE.UNKNOWN;
  private double targetPosition = 0;

  public ClimberModule(int _pheumaticsId, int motorId, int limitSwitchChannel) {
    this.lockerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    this.angleMotor = new TalonFX(motorId);
    this.angleController = new PIDController(0, 0, 0);

    this.angleMotor.config_kP(PRIMARY_PID_SLOT, 0.02, TIMEOUT_MS);
    this.angleMotor.config_kI(PRIMARY_PID_SLOT, 0.002, TIMEOUT_MS);

    this.limitSwitch = new DigitalInput(limitSwitchChannel);
  }

  public void setPower(double power) {
    this.angleMotor.set(ControlMode.PercentOutput, power);
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

  public double currentPosition() {
    return this.angleMotor.getSelectedSensorPosition(PRIMARY_PID_SLOT);
  }

  public void setPosition(double pos) {
    angleMotor.set(ControlMode.Position, pos);
  }

  // NOTE: move this to a command
  @Deprecated
  public void moveToPosition() {
    double error = angleController.calculate(
      angleMotor.getSelectedSensorPosition(),
      targetPosition
    );

    angleMotor.set(ControlMode.PercentOutput, error / targetPosition * 1);
  }
}
