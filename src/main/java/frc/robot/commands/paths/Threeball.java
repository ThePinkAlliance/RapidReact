// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoShootHood;
import frc.robot.TargetPackage;
import frc.robot.commands.AutoCollectGroup;
import frc.robot.commands.AutoHood;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.LeaveTarmack;
import frc.robot.commands.Navigate;
import frc.robot.commands.TargetTracking;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Threeball extends SequentialCommandGroup {

  class CollectorOn extends CommandBase {

    Collector m_collector;
    Supplier<Boolean> isFinished;

    public CollectorOn(Collector m_collector, Supplier<Boolean> isFinished) {
      this.m_collector = m_collector;
    }

    @Override
    public void execute() {
      m_collector.SetSpeedCollector(1);
      m_collector.setSolenoid(true);
    }

    @Override
    public boolean isFinished() {
      return this.isFinished.get();
    }
  }

  Navigate navCollect;

  /** Creates a new ThreeballRightBlue. */
  public Threeball(
    Base m_base,
    Shooter m_shooter,
    Collector m_collector,
    Limelight m_limelight,
    Hood m_hood
  ) {
    TargetPackage shooter_tp = new TargetPackage(
      Shooter.SHOOTER_Kp_AUTO_THREE_BALL,
      Shooter.SHOOTER_FF_AUTO_THREE_BALL,
      Hood.AUTO_SHOT_THREEBALL_TICK_COUNT,
      Shooter.SHOOTER_POWER_THREE_BALL
    );

    // Add your commands in the addCommands() call, e.g.
    addCommands(
      // in parallel: move to pick up ball
      new Navigate(m_base, 70, false)
      // in parallel: start collecting and move the hood to shooting position
        .alongWith(
          new AutoCollectGroup(m_collector, 1.6, true),
          new AutoHood(m_hood, shooter_tp.hoodPosition)
        ),
      // Shoot both balls
      new AutoShoot(m_shooter, m_collector, shooter_tp),
      new Navigate(m_base, 0, 75),
      new Navigate(m_base, 110, 0)
      .alongWith(new AutoCollectGroup(m_collector, 2.6, true)),
      new Navigate(m_base, 0, 25),
      new TargetTracking(m_base, m_limelight),
      new AutoShoot(m_shooter, m_collector, shooter_tp)
    );
  }
}
