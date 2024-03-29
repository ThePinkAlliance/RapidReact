// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HoodConstants;
import frc.robot.TargetPackage;
import frc.robot.TargetPackageFactory;
import frc.robot.commands.base.LimelightAlignAuto;
import frc.robot.commands.base.Navigate;
import frc.robot.commands.collector.AutoCollectGroup;
import frc.robot.commands.hood.AutoHood;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
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
      Limelight m_limelight) {
    // seconds needed (as seen during SLF testing) to collect the second ball.
    double autoCollectSeconds = 1.2;
    double shootSeconds = 2;
    double targetAcquireSeconds = 0.75;
    TargetPackage tp = TargetPackageFactory.getTwoBallPackage();
    boolean bUseLimelightInstead = false;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // in parallel: move to pick up ball
        new Navigate(m_base, 70, false)
            // in parallel: start collecting and move the hood to shooting position
            .alongWith(
                new AutoCollectGroup(m_collector, autoCollectSeconds, true),
                new AutoHood(m_hood, tp.hoodPosition)),
        // align before shooting
        new LimelightAlignAuto(m_base, m_limelight, targetAcquireSeconds),
        // Shoot both balls
        new AutoShoot(m_shooter, m_collector, m_hood, m_limelight, tp, shootSeconds, bUseLimelightInstead),
        new AutoHood(m_hood, HoodConstants.IDLE_TICK_COUNT));
  }
}