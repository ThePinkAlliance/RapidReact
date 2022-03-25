// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoShootHood;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.LeaveTarmack;
import frc.robot.commands.Navigate;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeballRightBlue extends SequentialCommandGroup {

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
  public ThreeballRightBlue(
    Base m_base,
    Shooter m_shooter,
    Collector m_collector,
    Hood m_hood
  ) {
    this.navCollect =
      new Navigate(
        m_base,
        LeaveTarmack.TARMAC_WIDTH / 2 + (LeaveTarmack.ROBOT_WIDTH / 2),
        0
      );

    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new Navigate(m_base, (LeaveTarmack.TARMAC_WIDTH / 2), false),
      new Navigate(m_base, 0, 15),
      navCollect.alongWith(
        new CollectorOn(m_collector, () -> this.navCollect.isFinished())
      ),
      new Navigate(m_base, 15, true),
      new Navigate(m_base, 0, -15),
      // the 10 is distance in inches this is a placeholder not intended for comp use
      new AutoShootHood(m_shooter, m_collector, m_hood, 100)
    );
  }
}
