// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tower;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class MoveTower extends CommandBase {

  int JOYSTICK_BUTTON;

  Collector m_collector;
  Joystick joystick;
  boolean bIntake;
  double shooterPowerCloseHigh;

  /**
   * NOTE bIntake won't be updated if this command is not being called repeatedly
   * 
   * @param m_collector
   * @param shooterPowerCloseHigh
   * @param JOYSTICK_BUTTON
   * @param gamepad_tower
   */
  public MoveTower(
      Collector m_collector,
      double shooterPowerCloseHigh,
      int JOYSTICK_BUTTON,
      Joystick gamepad_tower,
      boolean bIntake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_collector = m_collector;
    this.shooterPowerCloseHigh = shooterPowerCloseHigh;
    this.JOYSTICK_BUTTON = JOYSTICK_BUTTON;
    this.bIntake = bIntake;
    this.joystick = gamepad_tower;

    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bIntake == false)
      this.m_collector.SetSpeedTower(-Collector.TOWER_MOTOR_FULL_SPEED);
    else
      this.m_collector.SetSpeedTower(Collector.TOWER_MOTOR_FULL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_collector.SetSpeedTowerForOverride(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getRawButton(JOYSTICK_BUTTON) == false ? true : false;
  }
}
