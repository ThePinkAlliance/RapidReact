// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

public class Dashboard extends SubsystemBase {

  // One issue is this may not be great for performance.
  private HashMap<String, Object> data = new HashMap<String, Object>();
  private ShuffleboardTab station = Shuffleboard.getTab("Station");

  /** Creates a new Dashboard. */
  public Dashboard() {}

  public void putString(String title, Object value) {
    Object rawData = data.get(title);

    if (rawData == null) {
      station.add(title, value);
      data.put(title, value);
    } else {
      // station
      //   .getComponents()
      //   .forEach(
      //     (component) -> {
      //       if (component.getTitle().equals(title)) {
      //         component.withProperties()
      //       }
      //     }
      //   );
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
