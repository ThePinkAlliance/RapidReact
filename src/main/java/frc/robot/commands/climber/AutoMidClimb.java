// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ClimberModule;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Climbers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMidClimb extends SequentialCommandGroup {

  /** Creates a new AutoClimb. */
  public AutoMidClimb(Base base, Climbers climbers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveShortArms(
            climbers,
            ClimberModule.SHORT_ARM_MID_CLIMB_START,
            MoveShortArms.ARM_MOVE_UP),
        new ClimbDrive(base, climbers, 0, 0.4, false),
        new MoveShortArms(
            climbers,
            ClimberModule.SHORT_ARM_MID_CLIMB_FINISH,
            MoveShortArms.ARM_MOVE_DOWN));
  }
}
