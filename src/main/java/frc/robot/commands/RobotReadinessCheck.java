// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BooleanEntry;
import frc.robot.commands.base.ZeroPods;
import frc.robot.commands.battery.BatteryCheck;
import frc.robot.commands.hood.ResetHood;
import frc.robot.commands.pneumatics.PneumaticsCheck;
import frc.robot.commands.shooter.ShooterCheck;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RobotReadinessCheck extends SequentialCommandGroup {
  /** Creates a new ResetRobot. */
  public RobotReadinessCheck(Hood hood, Base base, Shooter shooter, Collector collector, Compressor compressor,
      BooleanEntry batterySufficient,
      BooleanEntry pneumaticsReady, BooleanEntry shooterReady) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetHood(hood), new ZeroPods(base),
        new PneumaticsCheck(compressor, pneumaticsReady), new ShooterCheck(shooter, collector, shooterReady));
  }
}
