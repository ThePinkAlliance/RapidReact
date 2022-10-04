// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import com.ThePinkAlliance.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BaseConstants;
import frc.robot.debugInfo.DebugInfo;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Dashboard;

public class Navigate extends CommandBase {

  Base base;

  double drive_kP = 1;
  double drive_kI = 0.5;
  double drive_kD = 0.002;

  boolean bBackwards = false;

  /**
   * kP:
   * kI:
   * kD: keep kD low otherwise your system could become unstable
   */

  PIDController straightController = new PIDController(
      drive_kP,
      drive_kI,
      drive_kD); // kP 0.27 kI 0.3 kD 0.002

  PIDController alignController = new PIDController(
      BaseConstants.navigateTurnGains.kP,
      BaseConstants.navigateTurnGains.kI,
      BaseConstants.navigateTurnGains.kD);

  double reduction = SdsModuleConfigurations.MK4I_L1.getDriveReduction();

  double targetAngle = 0;
  double targetInches = 0;

  DoubleLogEntry distanceInchesEntry;
  DoubleLogEntry angleEntry;

  /** Creates a new DriveStraight. */
  public Navigate(Base base, double targetInches, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.targetAngle = targetAngle;
    this.configureLogger();
    addRequirements(base);
  }

  public Navigate(Base base, double targetInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.configureLogger();
    addRequirements(base);
  }

  public Navigate(Base base, double targetInches, boolean bBackwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.bBackwards = bBackwards;
    this.configureLogger();
    addRequirements(base);
  }

  public Navigate(
      Base base,
      double targetInches,
      double targetAngle,
      boolean bBackwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.targetAngle = targetAngle;
    this.bBackwards = bBackwards;

    this.configureLogger();

    addRequirements(base);
  }

  private void configureLogger() {
    DataLogManager.start("logs-navigate");
    DataLog log = DataLogManager.getLog();

    this.distanceInchesEntry = new DoubleLogEntry(log, "distance-inches");
    this.angleEntry = new DoubleLogEntry(log, "current-angle");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.drive(new ChassisSpeeds());
    alignController.reset();
    alignController.enableContinuousInput(-180.0, 180.0);
    alignController.setTolerance(1);
    base.zeroGyro();

    straightController.reset();
    straightController.setTolerance(3);
    base.resetDriveMotors();

    straightController.setP(
        SmartDashboard.getNumber(
            Dashboard.DASH_NAVIGATE_kP_ANGLE_OFFSET,
            drive_kP));
    straightController.setI(
        SmartDashboard.getNumber(
            Dashboard.DASH_NAVIGATE_kI_ANGLE_OFFSET,
            drive_kI));
    straightController.setD(
        SmartDashboard.getNumber(
            Dashboard.DASH_NAVIGATE_kD_ANGLE_OFFSET,
            drive_kD));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x_output = 0.0;
    double x_power = 0.0;
    double turnPower = 0.0;

    // Drive
    if (targetInches != 0) {
      // double front_left_pos = Math.abs(
      // this.base.frontLeftModule.getDrivePosition()
      // );
      double front_right_pos = Math.abs(
          this.base.frontRightModule.getDrivePosition());

      double distance_traveled_inches = ((0.123825) * (front_right_pos / Base.FULL_TALON_ROTATION_TICKS)) *
          12.875;

      x_output = straightController.calculate(distance_traveled_inches, targetInches);
      x_power = (x_output / targetInches) * Base.MAX_VELOCITY_METERS_PER_SECOND;
      System.out.println("Navigate: " + x_power + ", Output" + x_output);

      this.distanceInchesEntry.append(distance_traveled_inches);
      this.distanceInchesEntry.setMetadata("target-distance=" + targetInches);
      DebugInfo.send("traveled", distance_traveled_inches);
    }
    // Turn: PID Controller using setpoint of zero
    else if (targetAngle != 0) {
      double currentAngle = base.getSensorYaw();
      double processVariable = Math.abs(targetAngle) - Math.abs(currentAngle);

      processVariable = Math.copySign(processVariable, targetAngle);

      double output = alignController.calculate(processVariable, 0);
      double limitedTurnPower = limitPower(
          output / 180,
          LimelightAlign.TRACKER_LIMIT_DEFAULT);
      turnPower = limitedTurnPower * Base.MAX_VELOCITY_METERS_PER_SECOND;

      this.angleEntry.append(currentAngle);
      this.angleEntry.setMetadata("target-angle=" + targetAngle);
    }

    if (bBackwards)
      x_power *= -1;
    ChassisSpeeds speeds = new ChassisSpeeds(x_power, 0, turnPower);
    base.drive(speeds);
  }

  private double limitPower(double currentPower, double limit) {
    double value = currentPower;
    if (Math.abs(currentPower) > limit)
      value = Math.copySign(limit, currentPower);
    System.out.println("limitPower: " + value + "; Original: " + currentPower);
    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(new ChassisSpeeds(0, 0, 0));
    base.resetDriveMotors();

    this.angleEntry.finish();
    this.distanceInchesEntry.finish();

    System.out.println(
        "END OF COMMAND: " +
            this.base.frontRightModule.getDrivePosition() +
            ", " +
            ((SdsModuleConfigurations.MK4I_L1.getDriveReduction()) *
                (this.base.frontRightModule.getDrivePosition() /
                    Base.FULL_TALON_ROTATION_TICKS)
                *
                Base.circumference)
            +
            ", " +
            "INTERRUPTED: " +
            interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean straightMet = straightController.atSetpoint();
    boolean turnMet = alignController.atSetpoint();
    System.out.println(
        "Straight Met: " + straightMet + "; turnMet: " + turnMet);

    // ONLY CHECK THE CONDITION FOR THE MOVEMENT WHOSE TARGET IS NOT ZERO
    if (targetInches == 0)
      return turnMet;
    else
      return straightMet;
  }
}
