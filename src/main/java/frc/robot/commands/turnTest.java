// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import java.util.function.DoubleSupplier;

public class turnTest extends CommandBase {

  double p = 2.3; // 0.009
  double i = 0.4;
  double d = 0.0;
  double setpoint = -90;

  PIDController alignController = new PIDController(p, i, d);
  Base base;

  NetworkTable debug = NetworkTableInstance.getDefault().getTable("debug");

  /** Creates a new LeaveBlueLeft_test. */
  public turnTest(Base base) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.reset();
    base.zeroGyro();
    alignController.enableContinuousInput(-180.0, 180.0);

    alignController.setTolerance(3);

    SmartDashboard.putNumber("VEL", Base.MAX_VELOCITY_METERS_PER_SECOND);
    SmartDashboard.putNumber("setpoint", setpoint);

    debug.getEntry("turn-target").getDouble(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = base.getSensorYaw();
    double error = alignController.calculate(currentAngle, setpoint);
    double power = (error / -180) * Base.MAX_VELOCITY_METERS_PER_SECOND;

    System.out.println(
      power + ", " + error + ", " + currentAngle + ", " + setpoint
    );

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("power")
      .setNumber(power);

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("currentAngle")
      .setNumber(currentAngle);

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("angle error")
      .setNumber(error);

    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, power);
    base.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // alignController.disableContinuousInput();
    //alignController.reset();

    base.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return alignController.atSetpoint();
  }
}
