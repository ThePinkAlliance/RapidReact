package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.ErrorMessages;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PathweaverFactory {

  private SwerveDriveKinematics m_kinematics;
  private Trajectory m_trajectory;
  private PIDController m_xController;
  private PIDController m_yController;
  private ProfiledPIDController m_thetaController;
  private Supplier<Pose2d> m_poseSupplier;

  private Gains m_xGains = new Gains(1, 0.5, 0.002);
  private Gains m_yGains = new Gains(1, 0.5, 0.002);
  private Gains m_thetaGains = new Gains(3.7, 0.5, 0.003);

  public PathweaverFactory(
      SwerveDriveKinematics m_kinematics,
      Supplier<Pose2d> m_poseSupplier,
      Trajectory m_trajectory,
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecond) {
    this.m_kinematics = m_kinematics;
    this.m_poseSupplier = m_poseSupplier;
    this.m_trajectory = m_trajectory;

    this.m_xController = new PIDController(this.m_xGains.kP, this.m_xGains.kI, this.m_xGains.kD);
    this.m_yController = new PIDController(this.m_yGains.kP, this.m_yGains.kI, this.m_yGains.kD);
    this.m_thetaController = new ProfiledPIDController(
        this.m_thetaGains.kP,
        this.m_thetaGains.kI,
        this.m_thetaGains.kD,
        new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSecond,
            // ? this might need to be squared
            maxAccelerationMetersPerSecond));
  }

  public PathweaverFactory(
      SwerveDriveKinematics m_kinematics,
      Supplier<Pose2d> m_poseSupplier,
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecond) {
    this.m_kinematics = m_kinematics;
    this.m_poseSupplier = m_poseSupplier;

    this.m_xController = new PIDController(this.m_xGains.kP, this.m_xGains.kI, this.m_xGains.kD);
    this.m_yController = new PIDController(this.m_yGains.kP, this.m_yGains.kI, this.m_yGains.kD);
    this.m_thetaController = new ProfiledPIDController(
        this.m_thetaGains.kP,
        this.m_thetaGains.kI,
        this.m_thetaGains.kD,
        new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSecond,
            // ? this might need to be squared
            maxAccelerationMetersPerSecond));
  }

  public void setTrajectory(Trajectory trajectory) {
    this.m_trajectory = trajectory;
  }

  public SwerveControllerCommand buildController(
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    ErrorMessages.requireNonNullParam(
        this.m_trajectory,
        "trajectory",
        "buildController");

    return new SwerveControllerCommand(
        m_trajectory,
        m_poseSupplier,
        m_kinematics,
        m_xController,
        m_yController,
        m_thetaController,
        outputModuleStates,
        requirements);
  }
}
