// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BaseConstants;
import frc.robot.subsystems.Base;

public class Drive extends CommandBase {

  private Base base;
  private Joystick js;

  SlewRateLimiter xLimiter = new SlewRateLimiter(12, 0);
  SlewRateLimiter yLimiter = new SlewRateLimiter(12, 0);
  SlewRateLimiter rotLimiter = new SlewRateLimiter(12, 0);

  /** Creates a new Drive. */
  public Drive(Base base, Joystick js) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;

    this.js = js;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetDriveMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * NOTE: strafing has an issue when driving as it gets closer to the end of the
   * joystick it gets exponentially faster
   */
  @Override
  public void execute() {
    double axis0x = js.getRawAxis(0);
    double axis1y = js.getRawAxis(1);
    double axis4rot = js.getRawAxis(4);

    // invert right joystick axis input to match clockwise, counter clockwise robot
    // behavior
    axis4rot *= -1;
    axis0x *= -1;
    axis1y *= -1;

    ChassisSpeeds speedObject = new ChassisSpeeds(
        xLimiter.calculate(modifyAxisLimited(axis1y) * Base.MAX_VELOCITY_METERS_PER_SECOND),
        yLimiter.calculate(modifyAxisLimited(axis0x) * Base.MAX_VELOCITY_METERS_PER_SECOND),
        rotLimiter.calculate(modifyAxisLimited(axis4rot) * Base.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    this.base.drive(speedObject);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxisLimited(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Cubing due to raw power until robot reaches competition weight.
    value = Math.copySign(value * value * value, value);

    // Limit the speed
    value = value * BaseConstants.MAX_SPEED;

    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this will set the target meter per second to 0
    // otherwise the robot will not stop when the command is terminated
    this.base.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
