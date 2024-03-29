// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CameraConstants;
import frc.robot.Constants;
import frc.robot.HoodConstants;
import frc.robot.TargetPackage;
import frc.robot.TargetPackageFactory;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonShooter extends CommandBase {
  PhotonCamera m_camera;
  Hood m_hood;
  Shooter m_shooter;
  int button_id;
  TargetPackage currentPackage;
  Joystick joystick;

  /** Creates a new PhotonShooter. */
  public PhotonShooter(
      PhotonCamera camera,
      Shooter m_shooter,
      Hood m_hood,
      Joystick joystick,
      int button_id) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_camera = camera;
    this.m_hood = m_hood;
    this.m_shooter = m_shooter;
    this.button_id = button_id;
    this.joystick = joystick;

    addRequirements(m_hood, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hood.setPID(
        HoodConstants.kGains.kP,
        HoodConstants.kGains.kI,
        HoodConstants.kGains.kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean low = joystick.getPOV() == Constants.JOYSTICK_POV_DOWN;
    boolean tarmac = joystick.getPOV() == Constants.JOYSTICK_POV_LEFT;
    boolean high = joystick.getPOV() == Constants.JOYSTICK_POV_UP;

    if (low) {
      currentPackage = TargetPackageFactory.getLowHubPackage();
      System.out.println("Low Hub Package");
    } else if (tarmac) {
      currentPackage = TargetPackageFactory.getTarmacPackage();
      System.out.println("Tarmac Package");
    } else if (high) {
      currentPackage = TargetPackageFactory.getHighHubPackage();
      System.out.println("High Hub Package");
    } else if (m_camera.getLatestResult().hasTargets()) {
      PhotonTrackedTarget pipelineResult = m_camera.getLatestResult().getBestTarget();

      /*
       * Try changing the resolution to 360 or somthing smaller because some
       * resolutions report inaccurate pitch estimations then others.
       */
      double distance = Units.metersToInches(
          PhotonUtils.calculateDistanceToTargetMeters(Units.feetToMeters(4.5),
              Units.feetToMeters(2.85),
              Units.degreesToRadians(CameraConstants.CAMERA_MOUNTED_ANGLE),
              Units.degreesToRadians(pipelineResult.getPitch())));

      currentPackage = TargetPackageFactory
          .getCustomPackage(distance);
      System.out.println("Custom Package Distance: " + distance + ", Kp: " + currentPackage.Kp + ", Kf: "
          + currentPackage.Kf + ", Hood: " + currentPackage.hoodPosition + ", Rpm: " + currentPackage.rpm
          + ", Raw Pitch: " + pipelineResult.getPitch() + ", x: " + Units
              .metersToInches(pipelineResult.getCameraToTarget().getX())
          + ", y: "
          + Units.metersToInches(pipelineResult.getCameraToTarget().getY()) + ", vector: " + new Vector2d(
              pipelineResult.getCameraToTarget().getX(), pipelineResult.getCameraToTarget().getY()));
    } else {
      currentPackage = TargetPackageFactory.getTarmacPackage();
    }

    m_hood.setPosition(currentPackage.hoodPosition);
    boolean ready = m_shooter.readyToShoot(currentPackage.rpm, 100);
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, ready);

    this.m_shooter.configKp(currentPackage.Kp);
    this.m_shooter.configFeedForward(currentPackage.Kf);
    this.m_shooter.commandRpm(currentPackage.rpm);

    SmartDashboard.putNumber(
        Dashboard.DASH_SHOOTER_VELOCITY,
        this.m_shooter.getMotorOutputPercent());

    SmartDashboard.putNumber(
        Dashboard.DASH_SHOOTER_RPMS,
        this.m_shooter.getMotorRpms());
    SmartDashboard.putNumber(Dashboard.DASH_SHOOTER_P, currentPackage.Kp);
    SmartDashboard.putNumber(Dashboard.DASH_SHOOTER_FF, currentPackage.Kf);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hood.disableCloseLoopControl();
    m_shooter.command(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !joystick.getRawButton(button_id);
  }
}
