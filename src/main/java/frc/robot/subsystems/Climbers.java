// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberModule;
import frc.robot.ClimberModule.SOLENOID_STATE;

public class Climbers extends SubsystemBase {

  // // The port range for the climbers is 40-59
  // public ClimberModule leftOutModule = new ClimberModule(
  //   40,
  //   41,
  //   42,
  //   43,
  //   true,
  //   0
  // );
  // public ClimberModule leftInModule = new ClimberModule(
  //   44,
  //   45,
  //   46,
  //   47,
  //   true,
  //   0
  // );
  // public ClimberModule rightOutModule = new ClimberModule(48, 49, 50, 51, 0);
  // public ClimberModule rightInModule = new ClimberModule(52, 53, 54, 55, 0);

  /** Creates a new Climbers. */
  public Climbers() {}

  public void commandAllPos(
    double leftOne,
    double leftTwo,
    double rightOne,
    double rightTwo
  ) {
    // leftOutModule.setPosition(leftOne);
    // leftInModule.setPosition(leftTwo);
    // rightOutModule.setPosition(rightOne);
    // rightInModule.setPosition(rightTwo);
  }

  /**
   * This is only recommended to be used in teleop and not auto.
   * @param leftOut
   * @param leftIn
   * @param rightOut
   * @param rightIn
   */
  public void commandAllPower(
    double leftOut,
    double leftIn,
    double rightOut,
    double rightIn
  ) {
    // dividing the joystick values by 2.5 will max power at 40% while keeping the joystick scaling smooth.
    leftOut = leftOut / 2.7;
    leftIn = leftIn / 2.7;
    rightOut = rightOut / 2.7;
    rightIn = rightIn / 2.7;

  //   leftOutModule.setPower(leftOut);
  //   leftInModule.setPower(leftIn);
  //   rightOutModule.setPower(rightOut);
  //   rightInModule.setPower(rightIn);

  //   if (leftInModule.contactedPole() && leftIn > 0) {
  //     leftInModule.setSolenoidState(SOLENOID_STATE.LOCKED);
  //   }

  //   if (leftOutModule.contactedPole() && leftOut > 0) {
  //     leftInModule.setSolenoidState(SOLENOID_STATE.LOCKED);
  //   }

  //   if (rightInModule.contactedPole() && rightIn > 0) {
  //     leftInModule.setSolenoidState(SOLENOID_STATE.LOCKED);
  //   }

  //   if (rightOutModule.contactedPole() && rightOut > 0) {
  //     leftInModule.setSolenoidState(SOLENOID_STATE.LOCKED);
  //   }
 }

  public void setAllSolenoid(SOLENOID_STATE state) {
    // leftOutModule.setSolenoidState(state);
    // leftInModule.setSolenoidState(state);
    // rightOutModule.setSolenoidState(state);
    // rightInModule.setSolenoidState(state);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
