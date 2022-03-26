// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

  // public final double
  public final double REV_TICKS_PER_REV = 4096;
  public final double MAX_HOOD_WIDTH_INCHES = 5.582;
  public final double HOOD_PARREL_SHOOTER = 2.4;
  public final double MAX_HOOD_HEIGHT_INCHES = 7.159;
  public final double HOOD_LENGTH_INCHES = 5.9;
  public final double HOOD_WHEEL_CIRCUMFERENCE = 2.5132741229;

  private final double HOOD_LENGTH_X = 2.5;
  private final double HOOD_LENGTH_Y = 4.25;
  private final double MAX_HOOD_POSITION = -795;
  private final double MAX_HOOD_SHOOTER_DIFF_X = 10.5;
  private final double HOOD_DIFF_WIDTH_INCHES_PER_TICK =
    HOOD_LENGTH_X / MAX_HOOD_POSITION;
  private final int HOOD_MOTOR = 31;

  private CANSparkMax hoodMotor;
  private SparkMaxPIDController hoodPid;
  private RelativeEncoder hoodEncoder;

  /** Creates a new Hood. */
  public Hood() {
    hoodMotor = new CANSparkMax(HOOD_MOTOR, MotorType.kBrushless);

    // configure the hood motor and the encoder
    this.hoodEncoder = this.hoodMotor.getEncoder();

    this.hoodPid = this.hoodMotor.getPIDController();

    this.hoodPid.setOutputRange(-0.5, 0.5);

    // this.hoodPid.setP(0);
    // this.hoodPid.setI(0);
    // this.hoodPid.setD(0);
    // this.hoodPid.setFF(0);

    this.hoodMotor.setSmartCurrentLimit(30);
  }

  public double hoodDesiredTicks(double angle) {
    return (
      (
        (Math.tan(angle) * (HOOD_LENGTH_X - HOOD_PARREL_SHOOTER)) /
        HOOD_WHEEL_CIRCUMFERENCE
      ) *
      REV_TICKS_PER_REV
    );
  }

  public void setPID(double p, double i, double d, double ff) {
    this.hoodPid.setP(p);
    this.hoodPid.setI(i);
    this.hoodPid.setD(d);
    this.hoodPid.setFF(ff);
  }

  public SparkMaxPIDController hoodPidController() {
    return hoodMotor.getPIDController();
  }

  public void commandHood(double power) {
    this.hoodMotor.set(power);
  }

  public double getHoodTicks() {
    return hoodEncoder.getPosition();
  }

  public void resetHoodEncoder() {
    this.hoodEncoder.setPosition(0);
  }

  public void setPosition(double ticks) {
    double rotations = ticks / REV_TICKS_PER_REV;

    this.hoodPid.setReference(rotations, ControlType.kPosition);
  }

  public double getHoodAngle() {
    double currentHeight =
      HOOD_PARREL_SHOOTER +
      (
        HOOD_WHEEL_CIRCUMFERENCE *
        (hoodEncoder.getPosition() / REV_TICKS_PER_REV)
      );

    double currentWidth =
      MAX_HOOD_SHOOTER_DIFF_X -
      (hoodEncoder.getPosition() * HOOD_DIFF_WIDTH_INCHES_PER_TICK);

    // * to properly calculate angle of the hood its opposite / adjacent
    // ? Make sure to rework the hood distance system and PLEASE measure from the inner circular area of the flywheel shaft in the cad
    return (Math.tan(currentHeight / currentWidth) * (180 / Math.PI));
  }

  public void resetPID() {
    this.hoodPid.setReference(0, ControlType.kCurrent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (this.hoodMotor != null && this.hoodEncoder != null) {
      SmartDashboard.putNumber("hood power", this.hoodMotor.get());
      SmartDashboard.putNumber(
        "hood position",
        this.hoodEncoder.getPosition() * REV_TICKS_PER_REV
      );
      SmartDashboard.putNumber(
        "hood position raw",
        this.hoodEncoder.getPosition()
      );
      SmartDashboard.putNumber("hood angle", getHoodAngle());
      SmartDashboard.putNumber("hood velocity", this.hoodEncoder.getVelocity());
    }
  }
}
