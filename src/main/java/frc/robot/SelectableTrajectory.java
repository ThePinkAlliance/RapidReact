package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.io.IOException;
import java.nio.file.Path;

public class SelectableTrajectory {

  String name;
  String location;
  Trajectory trajectory;
  Command defualtCommand;
  ParallelCommandGroup CommandGroup;
  SwerveControllerCommand SwerveControllerCommand;

  public SelectableTrajectory(
    String name,
    String location,
    SwerveControllerCommand SwerveControllerCommand,
    ParallelCommandGroup Command
  ) {
    this.location = location;
    this.name = name;
    this.CommandGroup = Command;

    this.SwerveControllerCommand = SwerveControllerCommand;
    this.defualtCommand = SwerveControllerCommand.alongWith(Command);

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

  public SelectableTrajectory(String name, String location, Command command) {
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

  public Command getDefualtCommand() {
    return defualtCommand;
  }
}
