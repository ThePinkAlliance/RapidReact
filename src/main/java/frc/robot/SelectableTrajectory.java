package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Base;
import java.io.IOException;
import java.nio.file.Path;

public class SelectableTrajectory {

  String name;
  String location;
  Trajectory trajectory;
  Command defualtCommand = new EmptyCommand();
  SwerveControllerCommand SwerveControllerCommand;

  public SelectableTrajectory(
    String name,
    String location,
    SwerveControllerCommand SwerveControllerCommand,
    SequentialCommandGroup Command,
    Base m_base
  ) {
    this.location = location;
    this.name = name;

    this.SwerveControllerCommand = SwerveControllerCommand;
    this.defualtCommand =
      SwerveControllerCommand
        .beforeStarting(Command)
        .andThen(() -> m_base.drive(new ChassisSpeeds(0, 0, 0)));

    try {
      Path trajectoryPath = Filesystem
        .getDeployDirectory()
        .toPath()
        .resolve(location);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      // Sending it to the Driver Station however im not sure if it's a very
      // noticeable ill that check later
      DriverStation.reportError(
        "Unable to open trajectory: " + location,
        ex.getStackTrace()
      );

      // by making it fail loudly, we can catch it before a match starts
      throw new Error("Unable to open trajectory: " + location);
    }
  }

  public SelectableTrajectory(
    String name,
    String location,
    SwerveControllerCommand SwerveControllerCommand,
    ParallelCommandGroup Command,
    Base m_base
  ) {
    this.location = location;
    this.name = name;

    this.SwerveControllerCommand = SwerveControllerCommand;
    this.defualtCommand =
      SwerveControllerCommand
        .alongWith(Command)
        .andThen(() -> m_base.drive(new ChassisSpeeds(0, 0, 0)));

    try {
      Path trajectoryPath = Filesystem
        .getDeployDirectory()
        .toPath()
        .resolve(location);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      // Sending it to the Driver Station however im not sure if it's a very
      // noticeable ill that check later
      DriverStation.reportError(
        "Unable to open trajectory: " + location,
        ex.getStackTrace()
      );

      // by making it fail loudly, we can catch it before a match starts
      throw new Error("Unable to open trajectory: " + location);
    }
  }

  public SelectableTrajectory(String name, String location, Base m_base) {
    this.location = location;
    this.name = name;

    try {
      Path trajectoryPath = Filesystem
        .getDeployDirectory()
        .toPath()
        .resolve(location);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      // Sending it to the Driver Station however im not sure if it's a very
      // noticeable ill that check later
      DriverStation.reportError(
        "Unable to open trajectory: " + location,
        ex.getStackTrace()
      );

      // by making it fail loudly, we can catch it before a match starts
      throw new Error("Unable to open trajectory: " + location);
    }

    // this.SwerveControllerCommand =
    //   new SwerveControllerCommand(
    //     trajectory,
    //     m_base::getPose,
    //     m_base.kinematics,
    //     // this is the x axis PID Controller
    //     new PIDController(
    //       SmartDashboard.getNumber("kP-X", 0),
    //       SmartDashboard.getNumber("kI-X", 0),
    //       SmartDashboard.getNumber("kD-X", 0)
    //     ),
    //     // this is the y axis PID Contrller
    //     new PIDController(
    //       SmartDashboard.getNumber("kP-Y", 0),
    //       SmartDashboard.getNumber("kI-Y", 0),
    //       SmartDashboard.getNumber("kD-Y", 0)
    //     ),
    //     new ProfiledPIDController(
    //       SmartDashboard.getNumber("theta-P", 0),
    //       SmartDashboard.getNumber("theta-I", 0),
    //       SmartDashboard.getNumber("theta-D", 0),
    //       new TrapezoidProfile.Constraints(
    //         Base.MAX_VELOCITY_METERS_PER_SECOND,
    //         Base.MAX_ACCELERATION_METERS_PER_SECOND
    //       )
    //     ),
    //     states -> {
    //       m_base.setStates(states);
    //     },
    //     m_base
    //   );

    this.defualtCommand =
      SwerveControllerCommand.andThen(
        () -> m_base.drive(new ChassisSpeeds(0, 0, 0))
      );
  }

  public SelectableTrajectory(
    String name,
    Trajectory trajectory,
    SwerveControllerCommand SwerveControllerCommand,
    ParallelCommandGroup Command,
    Base m_base
  ) {
    this.trajectory = trajectory;
    this.name = name;

    this.SwerveControllerCommand = SwerveControllerCommand;
    this.defualtCommand =
      SwerveControllerCommand
        .alongWith(Command)
        .andThen(() -> m_base.drive(new ChassisSpeeds(0, 0, 0)));

    try {
      Path trajectoryPath = Filesystem
        .getDeployDirectory()
        .toPath()
        .resolve(location);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      // Sending it to the Driver Station however im not sure if it's a very
      // noticeable ill that check later
      DriverStation.reportError(
        "Unable to open trajectory: " + location,
        ex.getStackTrace()
      );

      // by making it fail loudly, we can catch it before a match starts
      throw new Error("Unable to open trajectory: " + location);
    }
  }

  public SelectableTrajectory(
    String name,
    Trajectory trajectory,
    SwerveControllerCommand SwerveControllerCommand,
    Base m_base
  ) {
    this.trajectory = trajectory;
    this.name = name;

    this.SwerveControllerCommand = SwerveControllerCommand;
    this.defualtCommand =
      SwerveControllerCommand.andThen(
        () -> m_base.drive(new ChassisSpeeds(0, 0, 0))
      );
  }

  public SelectableTrajectory(String name, Trajectory trajectory, Base m_base) {
    this.trajectory = trajectory;
    this.name = name;

    // this.SwerveControllerCommand =
    //   new SwerveControllerCommand(
    //     trajectory,
    //     m_base::getPose,
    //     m_base.kinematics,
    //     // this is the x axis PID Controller
    //     new PIDController(
    //       SmartDashboard.getNumber("kP-X", 0),
    //       SmartDashboard.getNumber("kI-X", 0),
    //       SmartDashboard.getNumber("kD-X", 0)
    //     ),
    //     // this is the y axis PID Contrller
    //     new PIDController(
    //       SmartDashboard.getNumber("kP-Y", 0),
    //       SmartDashboard.getNumber("kI-Y", 0),
    //       SmartDashboard.getNumber("kD-Y", 0)
    //     ),
    //     new ProfiledPIDController(
    //       SmartDashboard.getNumber("theta-P", 0),
    //       SmartDashboard.getNumber("theta-I", 0),
    //       SmartDashboard.getNumber("theta-D", 0),
    //       new TrapezoidProfile.Constraints(
    //         Base.MAX_VELOCITY_METERS_PER_SECOND,
    //         Base.MAX_ACCELERATION_METERS_PER_SECOND
    //       )
    //     ),
    //     states -> {
    //       m_base.setStates(states);
    //     },
    //     m_base
    //   );

    this.defualtCommand =
      SwerveControllerCommand.andThen(
        () -> m_base.drive(new ChassisSpeeds(0, 0, 0))
      );
  }

  public SelectableTrajectory(String name, Command command) {
    this.name = name;

    this.defualtCommand = command;
  }

  public SelectableTrajectory(String name, String location) {
    this.location = location;
    this.name = name;

    try {
      Path trajectoryPath = Filesystem
        .getDeployDirectory()
        .toPath()
        .resolve(location);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      // Sending it to the Driver Station however im not sure if it's very noticeable. ill that check later
      DriverStation.reportError(
        "Unable to open trajectory: " + location,
        ex.getStackTrace()
      );

      // by making it fail loudly, we can catch it before a match starts
      throw new Error("Unable to open trajectory: " + location);
    }
  }

  public Command getDefualtCommand() {
    return defualtCommand;
  }
}

class EmptyCommand extends CommandBase {}
