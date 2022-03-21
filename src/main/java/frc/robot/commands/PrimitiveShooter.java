// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PrimitiveShooter extends CommandBase {

  private Shooter shooter;
  private Joystick joystick;
  private int button_id;

  /** Creates a new PrimimitveShooter. */
  public PrimitiveShooter(Shooter shooter, Joystick joystick, int button_id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.button_id = button_id;
    this.joystick = joystick;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pwr = joystick.getRawAxis(5);

    shooter.command(-1);
    shooter.commandHood(MathUtil.clamp(pwr, -0.4, 0.4));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.command(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getRawButtonReleased(button_id);
  }
}
