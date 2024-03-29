// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Dashboard;

public class MoveLongArms extends CommandBase {

  public static double ARM_MOVE_UP = -1;
  public static double ARM_MOVE_DOWN = 0.5;
  Climbers climbers;
  double position;
  double power;

  public MoveLongArms(Climbers climbers, double position, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbers = climbers;
    this.position = position;
    this.power = power;

    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber(
    // Dashboard.DASH_CLIMBER_LONG_ARM_POSITION,
    // climbers.longClimberModule.getPosition());
    this.climbers.longClimberModule.moveArms(ARM_MOVE_UP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.climbers.longClimberModule.moveArms(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition = this.climbers.longClimberModule.getPosition();
    return (Math.abs(currentPosition) > Math.abs(this.position));
  }
}
