// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BaseConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Limelight;

public class LimelightAlignAuto extends CommandBase {

  Base base;
  Limelight limelight;
  Joystick joystick;
  Timer timer;

  PIDController alignController = new PIDController(
      BaseConstants.targetTrackerGains.kP,
      BaseConstants.targetTrackerGains.kI,
      BaseConstants.targetTrackerGains.kD);

  private final double MAX_TIME = 0.75; // Arbitrary
  double maxSecondsToAcquireTarget = MAX_TIME;
  private final double ANGLE_TOLERANCE = 0.5; // Tuned at SLF
  public static final double TRACKER_LIMIT_DEFAULT = 0.45; // Tuned at SLF
  public static final double TRACKER_OFFSET = -2;
  private double setPoint = 0;

  public LimelightAlignAuto(
      Base baseSubsystem,
      Limelight limelightSubsystem,
      double maxSecondsToAcquireTarget) {
    this.base = baseSubsystem;
    this.limelight = limelightSubsystem;
    this.maxSecondsToAcquireTarget = maxSecondsToAcquireTarget;
    this.timer = new Timer();

    addRequirements(baseSubsystem, limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.reset();
    alignController.enableContinuousInput(-180.0, 180.0);
    alignController.setTolerance(this.ANGLE_TOLERANCE);

    alignController.setP(
        SmartDashboard.getNumber(
            Dashboard.DASH_TARGET_TRACKER_KP,
            BaseConstants.targetTrackerGains.kP));

    alignController.setI(
        SmartDashboard.getNumber(
            Dashboard.DASH_TARGET_TRACKER_KI,
            BaseConstants.targetTrackerGains.kI));

    alignController.setD(
        SmartDashboard.getNumber(
            Dashboard.DASH_TARGET_TRACKER_KD,
            BaseConstants.targetTrackerGains.kD));

    base.zeroGyro();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean availableTarget = limelight.isTarget();
    if (availableTarget == true) {
      double offset = 0;

      double output = alignController.calculate(
          limelight.getOffset(),
          setPoint + offset);
      output = limitAzimuthPower(output / 180);
      double power = output * Base.MAX_VELOCITY_METERS_PER_SECOND;

      System.out.println(
          "p: " +
              alignController.getP() +
              ", i: " +
              alignController.getI() +
              ", d: " +
              alignController.getD());

      ChassisSpeeds speeds = new ChassisSpeeds(0, 0, power);
      base.drive(speeds);
    }
  }

  private double limitAzimuthPower(double currentPower) {
    double value = currentPower;
    double limit = LimelightAlign.TRACKER_LIMIT_DEFAULT; // SmartDashboard.getNumber(Dashboard.BASE_ALIGN_LIMIT,
                                                         // LimelightAlign.TRACKER_LIMIT_DEFAULT);
    if (Math.abs(currentPower) > limit)
      value = Math.copySign(limit, currentPower);
    System.out.println(
        "limitAzimuthPower: " + value + "; Original: " + currentPower);
    return value;
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
