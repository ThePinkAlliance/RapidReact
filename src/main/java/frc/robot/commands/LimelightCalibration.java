// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TargetPackageFactory;
import frc.robot.subsystems.Limelight;

public class LimelightCalibration extends CommandBase {
  Limelight m_limelight;

  /** Creates a new LimelightCalibration. */
  public LimelightCalibration(Limelight m_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_limelight = m_limelight;

    addRequirements(m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dist = m_limelight.calculateDistanceHypot();

    SmartDashboard.putNumber("Hypot Distance", dist);
    SmartDashboard.putNumber("Target Kp", TargetPackageFactory.getCustomPackage(dist).Kp);
    SmartDashboard.putNumber("Target Kf", TargetPackageFactory.getCustomPackage(dist).Kf);
    SmartDashboard.putNumber("Target rpm", TargetPackageFactory.getCustomPackage(dist).rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
