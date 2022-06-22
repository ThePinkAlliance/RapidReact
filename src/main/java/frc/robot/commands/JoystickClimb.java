// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ClimberModule;
import frc.robot.Constants;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Dashboard;

public class JoystickClimb extends CommandBase {

  private Climbers climbers;
  private Joystick joystick;

  private final double BUMPER_DEADZONE = 0.5;
  private final double MIN_SHORT_CLIMBER_POSITION = 6240; //needs to be checked
  private final double MAX_SHORT_CLIMBER_POSITION = 213077.3; //needs to be checked
  private final double MIN_LONG_CLIMBER_POSITION = 6240; //needs to be checked
  private final double MAX_LONG_CLIMBER_POSITION = 213077.3; //needs to be checked
  private final double MIN_CLIMBER_POSITION = 0;
  private final double MAX_CLIMBER_POSITION = -88692;

  enum engagedSides {
    IN,
    OUT,
  }

  engagedSides currentEngagedSides = engagedSides.OUT;

  /** Creates a new ManualClimb. */
  public JoystickClimb(Climbers climbers, Joystick js) {
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
    if (joystick.getRawButton(Constants.JOYSTICK_LEFT_BUMPER)) {
      climbers.openShortArms();
    }
    if (
      joystick.getRawAxis(Constants.JOYSTICK_LEFT_TRIGGER) >
      Math.abs(BUMPER_DEADZONE)
    ) {
      climbers.closeShortArms();
    }
    if (joystick.getRawButton(Constants.JOYSTICK_RIGHT_BUMPER)) {
      climbers.openLongArms();
    }
    if (
      joystick.getRawAxis(Constants.JOYSTICK_RIGHT_TRIGGER) >
      Math.abs(BUMPER_DEADZONE)
    ) {
      climbers.closeLongArms();
    }

    double leftYstick = joystick.getRawAxis(Constants.JOYSTICK_LEFT_Y_AXIS);
    double shortPosition = Math.abs(climbers.shortClimberModule.getPosition());
    double longPosition = Math.abs(climbers.longClimberModule.getPosition());
    /* Deadband gamepad, short climbers */
    if (Math.abs(leftYstick) < 0.10) {
      /* Within 10% of zero */
      leftYstick = 0;
    }
    leftYstick = Math.copySign(leftYstick * leftYstick, leftYstick);
    double limiter = SmartDashboard.getNumber(
      Dashboard.DASH_CLIMBER_LIMITER,
      ClimberModule.CLIMBER_LIMITER
    );

    double rightYstick = joystick.getRawAxis(Constants.JOYSTICK_RIGHT_Y_AXIS);
    /* Deadband gamepad, long climbers */
    if (Math.abs(rightYstick) < 0.10) {
      /* Within 10% of zero */
      rightYstick = 0;
    }
    rightYstick = Math.copySign(rightYstick * rightYstick, rightYstick);

    // Allows the climber to go up at full speed.
    // if (rightYstick < -0) {
    //   rightYstick = rightYstick * Math.abs(1.0);
    // } else {
    //   rightYstick = rightYstick * Math.abs(limiter);
    // }

    // if (leftYstick < -0) {
    //   leftYstick = leftYstick * Math.abs(1.0);
    // } else {
    //   leftYstick = leftYstick * Math.abs(limiter);
    // }

    // Limits the climber power to 50%
    rightYstick = rightYstick * Math.abs(limiter);
    leftYstick = leftYstick * Math.abs(limiter);

    // Climber min stop
    if (shortPosition <= MIN_CLIMBER_POSITION && leftYstick > 0) {
      leftYstick = 0;
    }

    if (longPosition <= MIN_CLIMBER_POSITION && rightYstick > 0) {
      rightYstick = 0;
    }

    // Climber backstop
    if (shortPosition >= MAX_CLIMBER_POSITION && leftYstick < -0) {
      leftYstick = 0;
    }

    if (longPosition >= MAX_CLIMBER_POSITION && rightYstick < -0) {
      rightYstick = 0;
    }

    climbers.longClimberModule.moveArms(rightYstick);
    climbers.shortClimberModule.moveArms(leftYstick);

    SmartDashboard.putBoolean(
      "short left",
      climbers.shortClimberModule.contactedLeftPole()
    );
    SmartDashboard.putBoolean(
      "short right",
      climbers.shortClimberModule.contactedRightPole()
    );

    SmartDashboard.putBoolean(
      "long left",
      climbers.longClimberModule.contactedLeftPole()
    );
    SmartDashboard.putBoolean(
      "long right",
      climbers.longClimberModule.contactedRightPole()
    );

    SmartDashboard.putNumber(
      Dashboard.DASH_CLIMBER_LONG_ARM_POSITION,
      climbers.longClimberModule.getPosition()
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_CLIMBER_SHORT_ARM_POSITION,
      climbers.shortClimberModule.getPosition()
    );
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
