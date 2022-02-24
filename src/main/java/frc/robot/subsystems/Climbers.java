// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberModule;
import frc.robot.ClimberModule.SOLENOID_STATE;

public class Climbers extends SubsystemBase {

  // The port range for the climbers is 40-49
  public ClimberModule leftOutModule = new ClimberModule(40, 41);
  public ClimberModule leftInModule = new ClimberModule(42, 43);
  public ClimberModule rightOutModule = new ClimberModule(44, 45);
  public ClimberModule rightInModule = new ClimberModule(46, 47);

  /** Creates a new Climbers. */
  public Climbers() {}

  public void commandAllPos(
    double leftOne,
    double leftTwo,
    double rightOne,
    double rightTwo
  ) {
    leftOutModule.setPosition(leftOne);
    leftInModule.setPosition(leftTwo);
    rightOutModule.setPosition(rightOne);
    rightInModule.setPosition(rightTwo);
  }

  public void commandAllPower(
    double leftOut,
    double leftIn,
    double rightOut,
    double rightIn
  ) {
    leftOutModule.setPower(leftOut);
    leftInModule.setPower(leftIn);
    rightOutModule.setPower(rightOut);
    rightInModule.setPower(rightIn);
  }

  public void commandAllPower(
    double leftOut,
    double leftIn,
    double rightOut,
    double rightIn,
    double limit
  ) {
    // This massive if statement is to cap the power values if they exceed the set limit.
    if (leftOut > limit) {
      leftOut = limit;
    } else if (leftIn > limit) {
      leftIn = limit;
    } else if (leftOut < -limit) {
      leftOut = -limit;
    } else if (leftIn < -limit) {
      leftIn = -limit;
    } else if (rightOut > limit) {
      rightOut = limit;
    } else if (rightIn > limit) {
      rightIn = limit;
    } else if (rightOut < -limit) {
      rightOut = -limit;
    } else if (rightIn < -limit) {
      rightIn = -limit;
    }

    leftOutModule.setPower(leftOut);
    leftInModule.setPower(leftIn);
    rightOutModule.setPower(rightOut);
    rightInModule.setPower(rightIn);
  }

  public void setAllSolenoid(SOLENOID_STATE state) {
    leftOutModule.setSolenoidState(state);
    leftInModule.setSolenoidState(state);
    rightOutModule.setSolenoidState(state);
    rightInModule.setSolenoidState(state);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
