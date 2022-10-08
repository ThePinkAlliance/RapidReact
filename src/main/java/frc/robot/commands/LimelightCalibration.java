// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.swervelib.ZeroState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TargetPackage;
import frc.robot.TargetPackageFactory;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightLedMode;

public class LimelightCalibration extends CommandBase {
  Limelight m_limelight;
  // Base m_base;

  /** Creates a new LimelightCalibration. */
  public LimelightCalibration(Limelight m_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_limelight = m_limelight;
    // this.m_base = m_base;

    addRequirements(m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setLedState(LimelightLedMode.FORCE_ON);
    // this.m_base.setPodZeroStates(ZeroState.COAST);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dist = m_limelight.calculateDistanceHypot();
    double unmoddedDistance = m_limelight.calculateUnmodifiedDistance();
    TargetPackage target = TargetPackageFactory.getCustomPackage(dist);

    // this.m_logger.write(dist, unmoddedDistance, kP, kF, hoodPosition, rpm);

    SmartDashboard.putNumber("Hypot Distance", dist);
    SmartDashboard.putNumber("Raw Distance", unmoddedDistance);
    SmartDashboard.putNumber("Hood Position", target.hoodPosition);
    SmartDashboard.putNumber("Target Kp", target.Kp);
    SmartDashboard.putNumber("Target Kf", target.Kf);
    SmartDashboard.putNumber("Target rpm", target.rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setLedState(LimelightLedMode.FORCE_OFF);
    // m_base.setPodZeroStates(ZeroState.BRAKE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
