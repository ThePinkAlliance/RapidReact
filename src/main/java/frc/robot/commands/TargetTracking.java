// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Limelight;

public class TargetTracking extends CommandBase {
  Limelight limelight;
  PIDController alignController = new PIDController(3, 0.2, 0.002);
  Base base;

  double setpoint;


  public TargetTracking(Base baseSubsystem, Limelight limelightSubsystem, double targetAngle) {
    base = baseSubsystem;
    limelight = limelightSubsystem;

    addRequirements(baseSubsystem);
    addRequirements(limelightSubsystem);

    this.setpoint = targetAngle;
    this.base = baseSubsystem;
    this.limelight = limelightSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.reset();
    base.zeroGyro();
    alignController.enableContinuousInput(-180.0, 180.0);

    alignController.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean availableTarget = limelight.isTarget();
    double rotationAngle = limelight.getOffset();
    if (availableTarget == true) {
      setpoint = rotationAngle;
      double currentAngle = base.getSensorYaw();
      double power = MathUtil.clamp(
        alignController.calculate(currentAngle, setpoint),
        -0.7,
        0.7
      );

      NetworkTableInstance
        .getDefault()
        .getTable("debug")
        .getEntry("power")
        .setNumber(power);

      NetworkTableInstance
        .getDefault()
        .getTable("debug")
        .getEntry("angle error")
        .setNumber(alignController.calculate(currentAngle, setpoint));

      ChassisSpeeds speeds = new ChassisSpeeds(0, 0, power);
      base.drive(speeds);
    } else {
        double currentAngle = base.getSensorYaw();
        setpoint = currentAngle + 90;
        double power = MathUtil.clamp(
          alignController.calculate(currentAngle, setpoint),
          -0.7,
          0.7
        );

        NetworkTableInstance
          .getDefault()
          .getTable("debug")
          .getEntry("power")
          .setNumber(power);

        NetworkTableInstance
          .getDefault()
          .getTable("debug")
          .getEntry("angle error")
          .setNumber(alignController.calculate(currentAngle, setpoint));

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, power);
        base.drive(speeds);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
