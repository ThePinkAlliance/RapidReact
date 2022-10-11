// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.base.Navigate;
import frc.robot.commands.collector.AutoCollectGroup;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Threeball extends SequentialCommandGroup {

        /** Creates a new ThreeballRightBlue. */
        public Threeball(
                        Base m_base,
                        Shooter m_shooter,
                        Collector m_collector,
                        Limelight m_limelight,
                        Hood m_hood) {
                double autoCollectSeconds = 1.2;
                double autoCollectSecondsThirdBall = 2.6;
                double shootSeconds = 2;
                double shootSecondsThirdBall = 1.2;
                double targetAcquireSeconds = 0.75;
                // Add your commands in the addCommands() call, e.g.
                addCommands(
                                // in parallel: move to pick up ball
                                new Navigate(m_base, 75, false)
                                                // in parallel: start collecting and move the hood to shooting position
                                                .alongWith(
                                                                new AutoCollectGroup(m_collector, autoCollectSeconds,
                                                                                true)
                                                // new AutoHood(m_hood, shooter_tp.hoodPosition)
                                                ),
                                // Shoot both balls
                                new AutoShoot(
                                                m_shooter,
                                                m_collector,
                                                m_hood,
                                                m_limelight,
                                                null,
                                                shootSeconds,
                                                true),
                                new Navigate(m_base, 0, 95, false),
                                new Navigate(m_base, 105, 0, false)
                                                .alongWith(
                                                                new AutoCollectGroup(m_collector,
                                                                                autoCollectSecondsThirdBall, true)) // ,
                // new Navigate(m_base, 0, 25),
                // new LimelightAlign(m_base, m_limelight, targetAcquireSeconds)
                // new AutoShoot(m_shooter, m_collector, m_hood, m_limelight, null,
                // shootSecondsThirdBall, true)
                );
        }
}
