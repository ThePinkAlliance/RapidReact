// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class ShooterMonitor extends CommandBase {

  Shooter m_shooter;
  Tower m_tower;

  Joystick joystick;

  int JOYSTICK_BUTTON_SHOOT;
  int JOYSTICK_BUTTON_ACTIVATE_SHOOTER;

  /** Creates a new ShooterMonitor. */
  public ShooterMonitor(
    Shooter m_shooter,
    Tower m_tower,
    Joystick joystick,
    int JOYSTICK_BUTTON_SHOOT,
    int JOYSTICK_BUTTON_ACTIVATE_SHOOTER
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_tower = m_tower;
    this.m_shooter = m_shooter;
    this.joystick = joystick;

    this.JOYSTICK_BUTTON_SHOOT = JOYSTICK_BUTTON_SHOOT;
    this.JOYSTICK_BUTTON_ACTIVATE_SHOOTER = JOYSTICK_BUTTON_ACTIVATE_SHOOTER;

    addRequirements(m_shooter, m_tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean shoot = joystick.getRawButton(this.JOYSTICK_BUTTON_SHOOT);
    boolean toggle_shooter = joystick.getRawButton(
      this.JOYSTICK_BUTTON_ACTIVATE_SHOOTER
    );

    if (m_tower.ballDetected() && !shoot) {
      m_tower.commandMotor(0);
    } else if (!m_tower.ballDetected() && shoot && m_shooter.isActivate()) {
      m_tower.commandMotor(1);
    }

    if (toggle_shooter) {
      m_shooter.toggle(Constants.SHOOTER_CLOSE_HIGH);
    }

    if (shoot && m_tower.ballDetected() && m_shooter.isActivate()) {
      m_tower.commandMotor(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.command(0);
    m_tower.commandMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
