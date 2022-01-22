package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class SelectableTrajectory {
  String name;
  String location;
  Trajectory trajectory;

  public SelectableTrajectory(String name, String location) {
    this.location = location;
    this.name = name;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(location);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + location, ex.getStackTrace());
    }
  };
}
