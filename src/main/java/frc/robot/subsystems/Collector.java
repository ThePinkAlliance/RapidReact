// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

  public static int COLLECTOR_MOTOR_PORT = 10;
  public static double COLLECTOR_MOTOR_FULL_SPEED = 0.5;
  public static double TOWER_MOTOR_FULL_SPEED = 1;
  public static final double TOWER_SENSOR_TRIGGER_DISTANCE = 150.0; // millimeters
  private final int TOWER_MOTOR_PORT = 20;
  private final int SOLENOID_ID_CLOSE = 0;
  private final int SOLENOID_ID_OPEN = 6;

  // Collector
  private CANSparkMax collectorMotor;
  private DoubleSolenoid solenoid;
  private boolean collectorRunning = false;
  // Tower
  private TalonFX towerMotor;
  private Rev2mDistanceSensor ballSensor;
  private boolean towerOverride = false;
  private boolean towerReverse = false;

  public static Value COLLECTOR_OPEN = Value.kReverse;
  public static Value COLLECTOR_CLOSE = Value.kForward;

  /** Creates a new Collector. */
  public Collector() {
    // Collector
    this.collectorMotor = new CANSparkMax(Collector.COLLECTOR_MOTOR_PORT, MotorType.kBrushless);
    this.solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        SOLENOID_ID_OPEN,
        SOLENOID_ID_CLOSE);
    this.collectorMotor.setInverted(true);
    // Tower
    this.towerMotor = new TalonFX(TOWER_MOTOR_PORT);
    this.towerMotor.setInverted(true);
    this.towerMotor.setNeutralMode(NeutralMode.Brake);
    this.ballSensor = new Rev2mDistanceSensor(Port.kOnboard);
    this.ballSensor.setDistanceUnits(Unit.kMillimeters);
    this.ballSensor.setAutomaticMode(true);
    this.ballSensor.setEnabled(true);
    towerOverride = false;
  }

  public void enableTowerOverride() {
    this.towerOverride = true;
  }

  public void disableTowerOverride() {
    this.towerOverride = false;
  }

  public void dropCollector() {
    this.collectorMotor.set(1);
    this.solenoid.set(COLLECTOR_OPEN);
  }

  public void setSolenoid(Value on) {
    this.solenoid.set(on);
  }

  public void SetSpeedCollector(double speed) {
    collectorMotor.set(speed);
    if (speed != 0)
      collectorRunning = true;
    else
      collectorRunning = false;
  }

  public boolean ballDetected() {
    boolean bRangeValid = this.ballSensor.isRangeValid();
    double distance = this.ballSensor.getRange();
    boolean bDetected = distance < TOWER_SENSOR_TRIGGER_DISTANCE;
    // System.out.println(
    // "RANGE VALID: " +
    // bRangeValid +
    // ", Distance: " +
    // distance +
    // ", bDetected: " +
    // bDetected
    // );
    return (bDetected && bRangeValid);
  }

  // FIX: 100ms in total using visualVM
  public void SetSpeedTowerForOverride(double towerSpeed) {
    boolean ballFound = ballDetected();
    System.out.println("OUTPUT: " + collectorRunning + ", BALL: " + ballFound);
    // towerMotor.set(ControlMode.PercentOutput, TOWER_MOTOR_FULL_SPEED);
    if (this.towerOverride) {
      towerMotor.set(ControlMode.PercentOutput, towerSpeed);
    } else if (collectorRunning && ballFound == false) {
      towerMotor.set(ControlMode.PercentOutput, towerSpeed);
    } else {
      towerMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void SetSpeedTower(double power) {
    towerMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
