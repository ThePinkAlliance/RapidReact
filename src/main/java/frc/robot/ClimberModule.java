package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberModule {
  private Solenoid lockerSolenoid;
  private TalonFX angleMotor;
  private PIDController angleController;

  private double targetPosition = 0;

  public ClimberModule(int _pheumaticsId, int motorId) {
    this.lockerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    this.angleMotor = new TalonFX(motorId);
    this.angleController = new PIDController(0, 0, 0);
  }

  public void setPosition(double pos) {
    angleMotor.set(ControlMode.Position, 0);
  }

  // NOTE: move this to a command
  public void moveToPosition() {
    double error = angleController.calculate(angleMotor.getSelectedSensorPosition(), targetPosition);

    angleMotor.set(ControlMode.PercentOutput, error / targetPosition * 1);
  }
}
