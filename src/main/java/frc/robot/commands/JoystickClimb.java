// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ClimberModule.SOLENOID_SIDE;
import frc.robot.ClimberModule.SOLENOID_STATE;
import frc.robot.subsystems.Climbers;

public class JoystickClimb extends CommandBase {

  private Climbers climbers;
  private Joystick joystick;
  

  enum engagedSides {
    IN,
    OUT,
  }

  engagedSides currentEngagedSides = engagedSides.OUT;

  /** Creates a new ManualClimb. */
  public JoystickClimb(
    Climbers climbers,
    Joystick js
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.climbers = climbers;
    this.joystick = js;

    addRequirements(climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbers.openAllLocks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getRawButton(Constants.JOYSTICK_BUTTON_A)) {
      climbers.openShortArms();
    }
    if (joystick.getRawButton(Constants.JOYSTICK_BUTTON_B)) {
      climbers.closeShortArms();
    }
    if (climbers.shortClimberModule.contactedLeftPole()) {
      climbers.shortClimberModule.setSolenoidState(SOLENOID_STATE.LOCKED, SOLENOID_SIDE.LEFT);
    }
    if (climbers.shortClimberModule.contactedRightPole()) {
      climbers.shortClimberModule.setSolenoidState(SOLENOID_STATE.LOCKED, SOLENOID_SIDE.RIGHT);
    } 
    if (joystick.getRawButton(Constants.JOYSTICK_BUTTON_X)) {
      climbers.openLongArms();
    }
    if (joystick.getRawButton(Constants.JOYSTICK_BUTTON_Y)) {
      climbers.openLongArms();
    }
    if (climbers.longClimberModule.contactedLeftPole()) {
      climbers.longClimberModule.setSolenoidState(SOLENOID_STATE.LOCKED, SOLENOID_SIDE.LEFT);
    }
    if (climbers.longClimberModule.contactedRightPole()) {
      climbers.longClimberModule.setSolenoidState(SOLENOID_STATE.LOCKED, SOLENOID_SIDE.RIGHT);
    }

    double leftYstick = joystick.getRawAxis(Constants.JOYSTICK_LEFT_Y_AXIS);
    /* Deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* Within 10% of zero */
			leftYstick = 0;
		}

    double targetPositionRotations = leftYstick * 237 * 2048;
    climbers.shortClimberModule.setPosition(targetPositionRotations);

    double rightYstick = joystick.getRawAxis(Constants.JOYSTICK_RIGHT_Y_AXIS);
    /* Deadband gamepad */
		if (Math.abs(rightYstick) < 0.10) {
			/* Within 10% of zero */
			rightYstick = 0;
		}

    targetPositionRotations = rightYstick * 237 * 2048;
    climbers.longClimberModule.setPosition(targetPositionRotations);
    SmartDashboard.putNumber("LONG ARM POSITION:", climbers.longClimberModule.getPosition());
    SmartDashboard.putNumber("SHORT ARM POSITION:", climbers.shortClimberModule.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
