// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BooleanEntry;
import frc.robot.TargetPackage;
import frc.robot.TargetPackageFactory;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class ShooterCheck extends CommandBase {
  BooleanEntry shooterReady;
  Shooter shooter;
  Collector collector;

  TargetPackage targetPackage;
  Timer timer;

  enum distances {
    LOW,
    TARMAC,
    HIGH,
    TWOBALL,
    CUSTOM
  }

  boolean isFinished = false;

  final double customDistance = 160;
  final double timerInterval = 3;
  final double shooterThreshold = 60;

  distances distance = distances.LOW;

  /** Creates a new ShooterCheck. */
  public ShooterCheck(Shooter shooter, Collector collector, BooleanEntry shooterReady) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.collector = collector;
    this.shooterReady = shooterReady;
    this.targetPackage = TargetPackageFactory.getLowHubPackage();
    this.timer = new Timer();

    addRequirements(shooter, collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.SetSpeedTower(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (distance == distances.LOW) {
      targetPackage = TargetPackageFactory.getLowHubPackage();
    } else if (distance == distances.TARMAC) {
      targetPackage = TargetPackageFactory.getTarmacPackage();
    } else if (distance == distances.HIGH) {
      targetPackage = TargetPackageFactory.getHighHubPackage();
    } else if (distance == distances.TWOBALL) {
      targetPackage = TargetPackageFactory.getTwoBallPackage();
    } else if (distance == distances.CUSTOM) {
      targetPackage = TargetPackageFactory.getCustomPackage(customDistance);
    }

    if (shooter.readyToShoot(targetPackage.rpm, shooterThreshold)) {
      timer.start();
    }

    if (timer.advanceIfElapsed(timerInterval)) {
      distance = nextDistance(distance);

      if (distance == distances.CUSTOM
          && targetPackage.rpm == TargetPackageFactory.getCustomPackage(customDistance).rpm) {
        isFinished = true;
      }
    }

    runShooter(targetPackage);
  }

  public distances nextDistance(distances distance) {
    if (distance == distances.LOW) {
      return distances.TARMAC;
    } else if (distance == distances.TARMAC) {
      return distances.HIGH;
    } else if (distance == distances.HIGH) {
      return distances.TWOBALL;
    } else if (distance == distances.TWOBALL) {
      return distances.CUSTOM;
    } else {
      return distances.CUSTOM;
    }
  }

  public void runShooter(TargetPackage targetPackage) {
    this.shooter.configFeedForward(targetPackage.Kf);
    this.shooter.configKp(targetPackage.Kp);
    this.shooter.commandRpm(targetPackage.rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.command(0);
    this.collector.SetSpeedTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
