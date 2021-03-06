// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberModule;
import frc.robot.ClimberModule.SOLENOID_STATE;

public class Climbers extends SubsystemBase {

  private final int LONG_PNEUMATIC_ID_1 = 1;
  private final int LONG_PNEUMATIC_ID_2 = 2; //

  private final int SHORT_PNEUMATIC_ID_1 = 3;
  private final int SHORT_PNEUMATIC_ID_2 = 5;  //@SLF, when switching to new pneumatic control module (PCM) 
                                               //it was noted that port 4 has a loose connector so 5 was used instead.

  // Long Climber Motor Id's
  private final int LONG_MOTOR_ONE_ID = 50;
  private final int LONG_MOTOR_TWO_ID = 51;

  // Long Climber Limit Switch Id's
  private final int LONG_LIMIT_LEFT_ID = 3;
  private final int LONG_LIMIT_RIGHT_ID = 1;

  // Short Climber Motor Id's
  private final int SHORT_MOTOR_ONE_ID = 53;
  private final int SHORT_MOTOR_TWO_ID = 52;

  // Short Climber Limit Switch Id's
  private final int SHORT_LIMIT_LEFT_ID = 0;
  private final int SHORT_LIMIT_RIGHT_ID = 2;

  public ClimberModule shortClimberModule;
  public ClimberModule longClimberModule;

  private Supplier<Double> longClimberSupplier = () -> 0.0;
  private Supplier<Double> shortClimberSupplier = () -> 0.0;

  /** Creates a new Climbers. */
  public Climbers() {
    // NOTE one of the modules will be inverted

    longClimberModule =
      new ClimberModule(
        LONG_PNEUMATIC_ID_1,
        LONG_PNEUMATIC_ID_2,
        LONG_MOTOR_ONE_ID,
        //LONG_MOTOR_CENTER_ID,
        LONG_MOTOR_TWO_ID,
        false,
        LONG_LIMIT_LEFT_ID,
        LONG_LIMIT_RIGHT_ID
      );

    shortClimberModule =
      new ClimberModule(
        SHORT_PNEUMATIC_ID_1,
        SHORT_PNEUMATIC_ID_2,
        SHORT_MOTOR_ONE_ID,
        //SHORT_MOTOR_CENTER_ID,
        SHORT_MOTOR_TWO_ID,
        true,
        SHORT_LIMIT_LEFT_ID,
        SHORT_LIMIT_RIGHT_ID
      );
  }

  public void openArmLocks(ClimberModule module) {
    module.setSolenoidState(SOLENOID_STATE.UNLOCKED);
  }

  public void openShortArms() {
    openArmLocks(shortClimberModule);
  }

  public void openLongArms() {
    openArmLocks(longClimberModule);
  }

  public void openAllLocks() {
    openLongArms();
    openShortArms();
  }

  public void closeArmLocks(ClimberModule module) {
    module.setSolenoidState(SOLENOID_STATE.LOCKED);
  }

  public void closeShortArms() {
    closeArmLocks(shortClimberModule);
  }

  public void closeLongArms() {
    closeArmLocks(longClimberModule);
  }

  public Supplier<Double> getLongClimberSupplier() {
    return longClimberSupplier;
  }

  public Supplier<Double> getShortClimberSupplier() {
    return shortClimberSupplier;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (longClimberModule != null && shortClimberModule != null) {
      longClimberSupplier = () -> longClimberModule.getPosition();
      shortClimberSupplier = () -> shortClimberModule.getPosition(); 
    }

  }
}
