// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tower;

public class EnableTower extends CommandBase {

  private Tower tower;
  private double power;
  private Joystick joystick;
  private boolean bIntake;

  /** Creates a new Shoot. */
  public EnableTower(Tower tower, double power, Joystick js, boolean bIntake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.tower = tower;
    this.power = power;
    this.joystick = js;
    this.bIntake = bIntake;

    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bIntake == false) tower.commandMotor(power); else tower.commandMotor(
      -power
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.commandMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean pressed = joystick.getRawButton(Constants.JOYSTICK_BUTTON_Y);

    if (pressed) {
      return false;
    } else {
      return true;
    }
  }
}
