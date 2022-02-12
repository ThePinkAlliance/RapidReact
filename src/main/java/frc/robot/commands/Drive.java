// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Base;
import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {

  private Base base;
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier rot;

  /** Creates a new Drive. */
  public Drive(
    Base base,
    DoubleSupplier x,
    DoubleSupplier y,
    DoubleSupplier rot
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;

    // Gamepad input to the drivetrain ->
    // realtime using DoubleSuppliers which values update
    this.x = x;
    this.y = y;
    this.rot = rot;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.base.drive(
        new ChassisSpeeds(
          modifyAxis(y.getAsDouble()) *
          Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
          modifyAxis(x.getAsDouble()) *
          Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
          modifyAxis(rot.getAsDouble()) *
          Constants.Base.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        )
      );
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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this will set the target meter per second to 0
    // other wise the robot will not stop when the command is terminated
    this.base.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
