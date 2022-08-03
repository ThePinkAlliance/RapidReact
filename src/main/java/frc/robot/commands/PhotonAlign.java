// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.core.joystick.Buttons;
import com.ThePinkAlliance.core.joystick.Joystick;
import com.ThePinkAlliance.core.joystick.PovType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BaseConstants;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Dashboard;
import org.photonvision.PhotonCamera;

public class PhotonAlign extends CommandBase {
  PhotonCamera m_camera;
  Base m_base;

  Joystick joystick;
  Buttons m_button;
  Timer timer;

  PIDController alignController = new PIDController(
      BaseConstants.targetTrackerGains.kP,
      BaseConstants.targetTrackerGains.kI,
      BaseConstants.targetTrackerGains.kD);

  private final double MAX_TIME = 0.75; // Arbitrary
  private final double ANGLE_TOLERANCE = 0.5; // Tuned at SLF
  public static final double TRACKER_LIMIT_DEFAULT = 0.45; // Tuned at SLF
  public static final double TRACKER_OFFSET = -2;
  private double setPoint = 0;

  double maxSecondsToAcquireTarget = MAX_TIME;

  /** Creates a new PhotonAlign. */
  public PhotonAlign(Base m_base, PhotonCamera m_camera, Joystick m_joystick, Buttons m_button) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_camera = m_camera;
    this.m_base = m_base;
    this.joystick = m_joystick;
    this.m_button = m_button;
    this.timer = new Timer();

    addRequirements(m_base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.reset();
    alignController.enableContinuousInput(-180.0, 180.0);
    alignController.setTolerance(this.ANGLE_TOLERANCE);

    alignController.setP(
        SmartDashboard.getNumber(
            Dashboard.DASH_TARGET_TRACKER_KP,
            BaseConstants.targetTrackerGains.kP));

    alignController.setI(
        SmartDashboard.getNumber(
            Dashboard.DASH_TARGET_TRACKER_KI,
            BaseConstants.targetTrackerGains.kI));

    alignController.setD(
        SmartDashboard.getNumber(
            Dashboard.DASH_TARGET_TRACKER_KD,
            BaseConstants.targetTrackerGains.kD));

    m_base.zeroGyro();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean right = joystick.povActivated(PovType.EAST);
    boolean left = joystick.povActivated(PovType.WEST);

    boolean availableTarget = m_camera.getLatestResult().hasTargets();

    if (availableTarget) {
      double offset = 0;
      if (right)
        offset = SmartDashboard.getNumber(
            Dashboard.DASH_LIMELIGHT_ANGLE_OFFSET,
            TRACKER_OFFSET);
      else if (left)
        offset = SmartDashboard.getNumber(
            Dashboard.DASH_LIMELIGHT_ANGLE_OFFSET,
            TRACKER_OFFSET) *
            -1.0;

      double output = alignController.calculate(
          m_camera.getLatestResult().getBestTarget().getYaw(),
          setPoint + offset);
      output = limitAzimuthPower(output / 180);
      double power = output * Base.MAX_VELOCITY_METERS_PER_SECOND;

      System.out.println(
          "p: " +
              alignController.getP() +
              ", i: " +
              alignController.getI() +
              ", d: " +
              alignController.getD());

      ChassisSpeeds speeds = new ChassisSpeeds(0, 0, power);
      m_base.drive(speeds);
    }
  }

  private double limitAzimuthPower(double currentPower) {
    double value = currentPower;
    double limit = LimelightAlign.TRACKER_LIMIT_DEFAULT;

    if (Math.abs(currentPower) > limit)
      value = Math.copySign(limit, currentPower);
    System.out.println(
        "limitAzimuthPower: " + value + "; Original: " + currentPower);
    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    m_base.drive(speeds);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (joystick != null) {
      return joystick.getButton(this.m_button).get() || timer.hasElapsed(MAX_TIME);
    } else {
      return alignController.atSetpoint() || timer.hasElapsed(MAX_TIME);
    }
  }
}
