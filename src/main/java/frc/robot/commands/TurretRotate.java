// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Joystick;

public class TurretRotate extends CommandBase {

  private Turret turret;
  private double power;
  private Joystick js;

  /** Creates a new Shoot. */
  public TurretRotate(Turret turret, Joystick joystick, double power) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.turret = turret;
    this.power = power;
    this.js = joystick;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.rotate(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.rotate(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (js.getRawButton(Constants.JOYSTICK_BUTTON_X))
       return false;
    else
       return true;
  }
}
