// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class Navigate extends CommandBase {

  Base base;

  double drive_kP = 1;
  double drive_kI = 0.5;
  double drive_kD = 0.002;

  double align_kP = 3.7;
  double align_kI = 0.5;
  double align_kD = 0.003;

  boolean bBackwards = false;

  /**
   * kP:
   * kI:
   * kD: keep kD low otherwise your system could become unstable
   */

  PIDController straightController = new PIDController(
    drive_kP,
    drive_kI,
    drive_kD
  ); // kP 0.27 kI 0.3 kD 0.002

  PIDController alignController = new PIDController(
    align_kP,
    align_kI,
    align_kD
  );

  // 0.122807
  double reduction = SdsModuleConfigurations.MK4I_L1.getDriveReduction();

  double targetAngle = 0;
  double targetInches = 0;

  /** Creates a new DriveStraight. */
  public Navigate(Base base, double targetInches, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.targetAngle = targetAngle;

    addRequirements(base);
  }

  public Navigate(Base base, double targetInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;

    addRequirements(base);
  }

  public Navigate(Base base, double targetInches, boolean bBackwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.bBackwards = bBackwards;
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.drive(new ChassisSpeeds());

    alignController.reset();
    straightController.reset();

    base.zeroGyro();
    base.resetDriveMotors();

    alignController.enableContinuousInput(-180, 180);

    alignController.setTolerance(0.5, 1.0);
    straightController.setTolerance(1.5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = base.getSensorYaw();

    double front_left_pos = Math.abs(
      this.base.frontLeftModule.getDrivePosition()
    );
    double front_right_pos = Math.abs(
      this.base.frontRightModule.getDrivePosition()
    );

    double front_left_rot =
      reduction * (front_left_pos / Base.FULL_TALON_ROTATION_TICKS);

    double front_right_rot =
      reduction * (front_right_pos / Base.FULL_TALON_ROTATION_TICKS);

    double front_left_inches = front_left_rot * Base.circumference;
    double front_right_inches = front_right_rot * Base.circumference;

    double distance_traveled_inches =
      ((0.123825) * (front_right_pos / Base.FULL_TALON_ROTATION_TICKS)) *
      12.875;

    double x_error = straightController.calculate(
      distance_traveled_inches,
      targetInches
    );

    // set the distance power using a similar method as the turn test
    double x_power = 0;

    if (targetInches != 0) {
      x_power = (x_error / targetInches) * Base.MAX_VELOCITY_METERS_PER_SECOND;
    }

    System.out.println(x_power + ", " + x_error);

    double angle_error = alignController.calculate(currentAngle, targetAngle);
    double theta_power =
      (angle_error / -180) * Base.MAX_VELOCITY_METERS_PER_SECOND;

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("x")
      .setNumber(x_power);

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("theta")
      .setNumber(x_power);

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("front_right_inches")
      .setNumber(front_right_inches);

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("front_left_inches")
      .setNumber(front_left_inches);

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("front right rot")
      .setNumber(front_right_rot);

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("front left rot")
      .setNumber(front_left_rot);

    SmartDashboard.putNumber("traveled", distance_traveled_inches);

    NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("yaw")
      .setNumber(base.getSensorYaw());
    if (bBackwards)
       x_power *= -1;
    ChassisSpeeds speeds = new ChassisSpeeds(x_power, 0, theta_power);

    base.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(new ChassisSpeeds(0, 0, 0));
    base.resetDriveMotors();

    System.out.println(
      "END OF COMMAND: " +
      this.base.frontRightModule.getDrivePosition() +
      ", " +
      (
        (SdsModuleConfigurations.MK4I_L1.getDriveReduction()) *
        (
          this.base.frontRightModule.getDrivePosition() /
          Base.FULL_TALON_ROTATION_TICKS
        ) *
        Base.circumference
      ) +
      ", " +
      "INTERRUPTED: " +
      interrupted
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return straightController.atSetpoint() && alignController.atSetpoint();
  }
}
