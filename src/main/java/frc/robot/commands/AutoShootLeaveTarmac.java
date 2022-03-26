// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ShooterConstants;
import frc.robot.TargetPackage;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootLeaveTarmac extends SequentialCommandGroup {

  public AutoShootLeaveTarmac(
    Base m_base,
    Shooter m_shooter,
    Hood m_hood,
    Collector m_collector
  ) {
    TargetPackage tp = new TargetPackage(ShooterConstants.kGains.kP, ShooterConstants.kGains.kF, Hood.HUB_SHOT_TICK_COUNT, Shooter.SHOOTER_POWER_HUB_HIGH);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoHood(m_hood, Hood.HUB_SHOT_TICK_COUNT),
      new AutoShoot(m_shooter, m_collector, tp, AutoShoot.ONE_BALL_MAX_TIME),
      new Navigate(m_base, LeaveTarmack.TRAVEL_DISTANCE, false).alongWith(
      new AutoHood(m_hood, tp.hoodPosition))
    );
  }
}
