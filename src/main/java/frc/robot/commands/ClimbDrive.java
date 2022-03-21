// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Climbers;

public class ClimbDrive extends CommandBase {

  Base base;
  Climbers climbers;
  Timer timer;

  double power;
  double align_kP = 3.7;
  double align_kI = 0.5;
  double align_kD = 0.003;

  boolean bBackwards = false;

  double MAX_ALLOWABLE_ROLL_BEFORE_CLIMB = 30;

  /**
   * kP:
   * kI:
   * kD: keep kD low otherwise your system could become unstable
   */

  PIDController alignController = new PIDController(
    align_kP,
    align_kI,
    align_kD
  );

  // 0.122807
  double reduction = SdsModuleConfigurations.MK4I_L1.getDriveReduction();

  double targetAngle = 0;
  double watchDogTime = 3;

  double powerLeft;
  double powerRight;

  /** Creates a new DriveStraight. */
  public ClimbDrive(
    Base base,
    Climbers climbers,
    double targetAngle,
    double power,
    boolean backwards
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.climbers = climbers;
    this.power = power;

    this.targetAngle = targetAngle;
    timer = new Timer();

    addRequirements(base, climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.drive(new ChassisSpeeds());
    alignController.reset();

    base.zeroGyro();
    base.resetDriveMotors();

    alignController.enableContinuousInput(-180, 180);

    alignController.setTolerance(0.5, 1.0);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean latchClimbers = base.getRoll() >= MAX_ALLOWABLE_ROLL_BEFORE_CLIMB;

    if (bBackwards) power *= -1;

    if (latchClimbers) this.climbers.closeShortArms();

    if (!latchClimbers) {
      this.powerLeft = power * Base.MAX_VELOCITY_METERS_PER_SECOND;
      this.powerRight = power * Base.MAX_VELOCITY_METERS_PER_SECOND;
    } else {
      this.powerLeft = 0;
      this.powerRight = 0;
    }

    System.out.println(
      "POWER LEFT: " +
      powerLeft +
      ", POWER RIGHT: " +
      powerRight +
      ", RIGHT POLE: " +
      this.climbers.shortClimberModule.contactedRightPole() +
      ", LEFT POLE: " +
      this.climbers.shortClimberModule.contactedLeftPole()
    );

    this.drive(this.powerLeft, this.powerRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(new ChassisSpeeds(0, 0, 0));
    base.resetDriveMotors();
    timer.stop();
  }

  public void drive(double left, double right) {
    SwerveModuleState[] states = base.getStates();

    SwerveModuleState frontLeftModuleState = states[0];
    SwerveModuleState frontRightModuleState = states[1];
    SwerveModuleState backLeftModuleState = states[2];
    SwerveModuleState backRightModuleState = states[3];

    frontLeftModuleState.speedMetersPerSecond = left;
    backLeftModuleState.speedMetersPerSecond = left;

    frontRightModuleState.speedMetersPerSecond = right;
    backRightModuleState.speedMetersPerSecond = right;

    base.setStates(
      frontLeftModuleState,
      frontRightModuleState,
      backLeftModuleState,
      backRightModuleState
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean value = false;
    if (
      climbers.shortClimberModule.getSolenoidState() ||
      timer.get() > watchDogTime
    ) value = true; else value = false;
    System.out.println(
      "ClimbDrive:  isFinished: " + value + " / " + timer.get()
    );
    return value;
  }
}
