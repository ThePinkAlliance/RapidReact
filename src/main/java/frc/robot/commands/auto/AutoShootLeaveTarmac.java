// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HoodConstants;
import frc.robot.TargetPackage;
import frc.robot.TargetPackageFactory;
import frc.robot.commands.base.Navigate;
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
public class AutoShootLeaveTarmac extends SequentialCommandGroup {

  public AutoShootLeaveTarmac(
      Base m_base,
      Shooter m_shooter,
      Hood m_hood,
      Collector m_collector,
      Limelight m_limelight) {
    TargetPackage tp = TargetPackageFactory.getHighHubPackage();
    boolean bUseLimelightInstead = false;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutoHood(m_hood, tp.hoodPosition),
        new AutoShoot(m_shooter, m_collector, m_hood, m_limelight, tp, AutoShoot.ONE_BALL_MAX_TIME,
            bUseLimelightInstead),
        new Navigate(m_base, LeaveTarmack.TRAVEL_DISTANCE, false),
        new AutoHood(m_hood, HoodConstants.IDLE_TICK_COUNT));
  }
}
