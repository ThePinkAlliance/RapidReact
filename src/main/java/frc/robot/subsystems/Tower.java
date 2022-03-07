// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import java.util.function.DoubleSupplier;

public class Tower extends SubsystemBase {

  /**
   * NOTE: sometimes i2c devices won't be constructed until a little while after the robot starts.
   */
  public static final double RGB_THRESHOLD = 230;
  private final int TOWER_MOTOR = 20;
  //private final I2C.Port port = I2C.Port.kOnboard;
  //private final ColorSensorV3 colorSensor = new ColorSensorV3(port);
  private final Rev2mDistanceSensor distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
  
  private TalonFX motor = new TalonFX(TOWER_MOTOR);
  public static final double TOWER_SENSOR_TRIGGER_DISTANCE = 10.0; //millimeters

  private DoubleSupplier red = () -> 0.0;
  private DoubleSupplier blue = () -> 0.0;

  /** Creates a new TempBase. */
  public Tower() {
    this.motor.setInverted(true);
    this.motor.setNeutralMode(NeutralMode.Brake);
    this.distOnboard.setDistanceUnits(Unit.kMillimeters);
    this.distOnboard.setAutomaticMode(true);
  }

  public boolean ballDetected() {
    boolean bRangeValid = this.distOnboard.isRangeValid();
    boolean bDetected =  this.distOnboard.GetRange() < TOWER_SENSOR_TRIGGER_DISTANCE;
    return (bRangeValid && bDetected);
  }

  public void commandMotor(double front) {
    this.motor.set(ControlMode.PercentOutput, front);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (colorSensor != null) {
    //   this.blue = () -> colorSensor.getBlue();
    //   this.red = () -> colorSensor.getRed();

    //   NetworkTableInstance
    //     .getDefault()
    //     .getTable("debug")
    //     .getEntry("red")
    //     .setNumber(colorSensor.getRed());
    //   NetworkTableInstance
    //     .getDefault()
    //     .getTable("debug")
    //     .getEntry("blue")
    //     .setNumber(colorSensor.getBlue());
    // }
  }
}
