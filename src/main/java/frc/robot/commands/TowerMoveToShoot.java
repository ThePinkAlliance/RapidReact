// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Tower;

public class TowerMoveToShoot extends CommandBase {

  private Tower tower;
  private double power;
  private double actualPower;
  private Joystick joystick;
  private int button;

  /** Creates a new TowerMoveToShoot command. */
  public TowerMoveToShoot(Tower tower, double power, Joystick js, int button) {
    
    this.tower = tower;
    this.power = power;
    this.joystick = js;
    this.actualPower = power;
    this.button = button;

    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //power can be overwritten via SmartDashboard, 
    //otherwise it uses the passed in power during command instantiation
    double dashPower = SmartDashboard.getNumber(
      Dashboard.DASH_SHOOTER_POWER,
      this.power);
    this.actualPower = dashPower;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //actuate the motor based on what the operator or programmer set for power
    tower.commandMotor(this.actualPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //turn off motor as command is completed
    tower.commandMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getRawButton(button) ? false : true;  //false when button still pressed, true when button not pressed 
  }
}
