// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BaseConstants;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Limelight;

public class TargetTracking extends CommandBase {

  Base base;
  Limelight limelight;
  Timer timer;
  PIDController alignController = new PIDController(
    BaseConstants.targetTrackerGains.kP,
    BaseConstants.targetTrackerGains.kI,
    BaseConstants.targetTrackerGains.kD
  );

  private final double MAX_TIME = 3;
  private final double ANGLE_TOLERANCE = 2;

  public TargetTracking(Base baseSubsystem, Limelight limelightSubsystem) {
    base = baseSubsystem;
    limelight = limelightSubsystem;
    timer = new Timer();

    addRequirements(baseSubsystem, limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.reset();
    base.zeroGyro();
    timer.reset();
    timer.start();
    alignController.enableContinuousInput(-180.0, 180.0);

    alignController.setP(
      SmartDashboard.getNumber(
        Dashboard.DASH_TARGET_TRACKER_KP,
        BaseConstants.targetTrackerGains.kP
      )
    );

    alignController.setI(
      SmartDashboard.getNumber(
        Dashboard.DASH_TARGET_TRACKER_KI,
        BaseConstants.targetTrackerGains.kI
      )
    );

    alignController.setD(
      SmartDashboard.getNumber(
        Dashboard.DASH_TARGET_TRACKER_KD,
        BaseConstants.targetTrackerGains.kD
      )
    );

    alignController.setTolerance(this.ANGLE_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean availableTarget = limelight.isTarget();
    double targetWithOffset = handleOverflow(
      limelight.getOffset() - base.getSensorYaw()
    );

    if (availableTarget == true) {
      double error = alignController.calculate(
        base.getSensorYaw(),
        targetWithOffset
      );
      double power = (error / -180) * Base.MAX_VELOCITY_METERS_PER_SECOND;

      SmartDashboard.putNumber("targetWithOffset", targetWithOffset);
      SmartDashboard.putNumber("power", power);

      ChassisSpeeds speeds = new ChassisSpeeds(0, 0, power);
      base.drive(speeds);
    }
  }

  public double handleOverflow(double angle) {
    if (angle > 180) {
      return (angle - 180);
    } else {
      return angle;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    base.drive(speeds);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return alignController.atSetpoint() || timer.hasElapsed(MAX_TIME);
  }
}
