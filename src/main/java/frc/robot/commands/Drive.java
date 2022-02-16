// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Base;
import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.JobHoldUntil;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Drive extends CommandBase {

  private Base base;
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier rot;
  private Joystick js;

  /** Creates a new Drive. */
  public Drive(
    Base base,
    DoubleSupplier x,
    DoubleSupplier y,
    DoubleSupplier rot,
    Joystick js
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;

    // Gamepad input to the drivetrain ->
    // realtime using DoubleSuppliers which values update
    this.x = x;
    this.y = y;
    this.rot = rot;
    this.js = js;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double axis0x = js.getRawAxis(0);
    double axis1y = js.getRawAxis(1);
    double axis4rot = js.getRawAxis(4);

    //invert right joystick axis input to match clockwise, counter clockwise robot behavior
    axis4rot *= -1;
    axis0x *= -1;
    axis1y *= -1;

    ChassisSpeeds speedObject = new ChassisSpeeds(
      modifyAxis(axis1y)   *  Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(axis0x)   *  Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(axis4rot) *  Constants.Base.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    );

    /*ChassisSpeeds speedObject = ChassisSpeeds.fromFieldRelativeSpeeds(
      modifyAxis(axis1y)   *  Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(axis0x)   *  Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(axis4rot) *  Constants.Base.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      base.getRotation()
    );*/
    //ChassisSpeeds speedObject = ChassisSpeeds.fromFieldRelativeSpeeds(
    //  modifyAxis(y.getAsDouble()) *
    //  Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
    //  modifyAxis(x.getAsDouble()) *
    //  Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
    //  modifyAxis(rot.getAsDouble()) *
    //  Constants.Base.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    //  base.getRotation()
    //);
  
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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Cubing due to raw power until robot reaches competition weight. 
    value = Math.copySign(value * value * value, value);

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
