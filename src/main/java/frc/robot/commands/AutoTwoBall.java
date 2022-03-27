// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HoodConstants;
import frc.robot.TargetPackage;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoBall extends SequentialCommandGroup {

  /** Creates a new ShootLeaveTarmac. */
  public AutoTwoBall(
    Base m_base,
    Shooter m_shooter,
    Collector m_collector,
    Hood m_hood,
    TargetPackage tp
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new AutoHood(m_hood,tp.hoodPosition),
      //in parallel: move to pick up ball
      new Navigate(m_base, 70, false)
      //in parallel: start collecting and move the hood to shooting position
        .alongWith(
          new AutoCollectGroup(m_collector, 1.6, true),
          new AutoHood(m_hood, tp.hoodPosition)
        ),
      //Shoot both balls
      new AutoShoot(m_shooter, m_collector, tp),
      new AutoHood(m_hood, HoodConstants.IDLE_TICK_COUNT)
    );
  }
}
