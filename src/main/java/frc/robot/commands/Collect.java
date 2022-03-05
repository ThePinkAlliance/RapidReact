// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collector;

public class Collect extends CommandBase {

  Collector m_collector;
  Joystick joystick;
  Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

  /** Creates a new Collect. */
  public Collect(Collector collecter, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collecter);

    this.m_collector = collecter;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set motor to speed set in constants so we can change
    this.m_collector.SetSpeed(Collector.COLLECTOR_MOTOR_FULL_SPEED);

    this.solenoid.set(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop motor so collector no longer runs
    this.m_collector.SetSpeed(0.0);
    this.solenoid.set(false);

    System.out.println("COMMAND COLLECT END: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean pressed = joystick.getRawButton(Constants.JOYSTICK_BUTTON_B);

    if (pressed) {
      return false;
    } else {
      return true;
    }
  }
}
