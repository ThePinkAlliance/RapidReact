// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

// ADDRESS FOR THE LIMELIGHT FEED: http://limelight.local:5801/

public class Limelight extends SubsystemBase {

  private boolean limelightLedOn = false;

  private Supplier<Double> horzontalOffset = () -> 0.0;
  private Supplier<Double> distanceSupplier = () -> 0.0;
  private Supplier<Double> hypotDistanceSupplier = () -> 0.0;
  private Supplier<Double> angleSupplier = () -> 0.0;

  private final double limelightLensHeight = 25; // 33.5; // this can change (in) will be static, should NEVER change
  private final double reflectiveTapeHeight = 102.375; // this is static (in) to CENTER of reflective tape
  private final double targetHeightDifference = (reflectiveTapeHeight - limelightLensHeight);

  ArrayList<Double> cachedHypotDistances = new ArrayList<Double>();

  private double lastHypotDistance = 0;

  double errorAccDistance = 0;
  double limelightMountedAngle = 51.5; // 50 // this can change a static number though once we have found it

  /** Creates a new Limelight. */
  public Limelight() {
    initLimelight(LimelightLedMode.FORCE_OFF);

    // Populate the cache
    cachedHypotDistances.add(0, 0.0);
    cachedHypotDistances.add(1, 0.0);
    cachedHypotDistances.add(2, 0.0);
    cachedHypotDistances.add(3, 0.0);
    cachedHypotDistances.add(4, 0.0);
    cachedHypotDistances.add(5, 0.0);
    cachedHypotDistances.add(6, 0.0);
    cachedHypotDistances.add(7, 0.0);
    cachedHypotDistances.add(8, 0.0);
    cachedHypotDistances.add(9, 0.0);
    cachedHypotDistances.add(10, 0.0);
  }

  public void initLimelight(LimelightLedMode mode) {
    NetworkTableInstance
        .getDefault()
        .getTable("limelight")
        .getEntry("ledMode")
        .setNumber(mode.get());
    NetworkTableInstance
        .getDefault()
        .getTable("limelight")
        .getEntry("camMode")
        .setNumber(0);
    NetworkTableInstance
        .getDefault()
        .getTable("limelight")
        .getEntry("pipeline")
        .setNumber(0);
  }

  public void setLedState(LimelightLedMode mode) {
    if (mode == LimelightLedMode.FORCE_ON) {
      limelightLedOn = true;
    } else {
      limelightLedOn = false;
    }
    initLimelight(mode); // Off or On
  }

  public boolean isTarget() {
    boolean targets = false;
    NetworkTable table = NetworkTableInstance
        .getDefault()
        .getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    double availableTargets = tv.getDouble(0.0);
    if (availableTargets == 1) {
      targets = true;
    } else {
      targets = false; // not necessary but for safety
    }
    return targets;
  }

  public double getOffset() {
    NetworkTable table = NetworkTableInstance
        .getDefault()
        .getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double offsetX = tx.getDouble(0.0);

    return offsetX;
  }

  public double calculateAccountedDistance() {
    // from documentation, the distance can be found using a fixed camera angle
    // distance = (height2 - height1) / tan(angle1 + angle2)
    // height1 is limelight elevation, height2 is target height
    // angle1 is limelight angle, angle2 is target angle
    // What we need: limelight angle on the robot, distance from center of limelight
    // lens to ground,
    // distance from height of the target to the floor

    NetworkTable table = NetworkTableInstance
        .getDefault()
        .getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");

    double offsetX = tx.getDouble(0.0) + horzontalOffset.get();

    double verticalOffsetAngle = ty.getDouble(0.0); // angle calculated by the limelight.

    double angleToGoalDeg = (limelightMountedAngle + verticalOffsetAngle);
    double angleToGoalRad = angleToGoalDeg * (3.14159 / 180.0);
    double error = 0.612649568;

    double distance = ((reflectiveTapeHeight - limelightLensHeight) /
        (Math.tan(angleToGoalRad)));

    // The following statements were used as ways to account for error that the
    // limelight gives from distance
    // There are a bunch of different statements to be more prescise at every
    // distance
    if (32.905 <= distance && distance <= 37.6) {
      error = 0.685521;
    }
    if (37.7 <= distance && distance <= 43.4) {
      error = 0.64395;
    }
    if (43.5 <= distance && distance <= 48.5) {
      error = 0.6303;
    }
    if (48.6 <= distance && distance <= 54.5) {
      error = 0.641166666666667;
    }
    if (54.6 <= distance && distance <= 62.5) {
      error = 0.609114583333333;
    }
    if (62.6 <= distance && distance <= 69.40) {
      error = 0.610555555555556;
    }
    if (69.41 <= distance && distance <= 76.43) {
      error = 0.611683333333333;
    }
    if (79.44 <= distance && distance <= 89.4) {
      error = 0.674242424242424;
    }
    if (89.41 <= distance && distance <= 91.2) {
      error = 0.623541666666667;
    }
    if (91.21 <= distance && distance <= 98.416) {
      error = 0.589606456;
    }
    if (98.417 <= distance && distance <= 162.5) {
      error = 0.555793264;
    }

    errorAccDistance = (distance / error);

    if (58.4 <= errorAccDistance && errorAccDistance <= 73.2) {
      errorAccDistance = errorAccDistance * 0.926887142;
    }
    if (58.4 <= errorAccDistance && errorAccDistance <= 73.2) {
      errorAccDistance = errorAccDistance * 0.926887142;
    }
    if (87.41 <= errorAccDistance && errorAccDistance <= 98.3) {
      errorAccDistance = errorAccDistance * 0.952870388;
    }
    if (162.51 <= errorAccDistance && errorAccDistance <= 188.4) {
      errorAccDistance = errorAccDistance * 0.908051108;
    }
    if (188.41 <= errorAccDistance && errorAccDistance <= 210.2) {
      errorAccDistance = errorAccDistance * 0.94147961;
    } else {
      errorAccDistance = distance / error;
    }

    errorAccDistance = errorAccDistance * 0.73;

    return errorAccDistance;
  }

  public boolean update(double next, double... vals) {
    int differences = 0;

    for (int i = 0; i < vals.length; i++) {
      double v = vals[i];

      if (v != next) {
        differences++;
      }
    }

    return differences >= 2;
  }

  public double calculateDistanceHypot() {
    double errorAccDistance = calculateAccountedDistance();
    double squared = ((targetHeightDifference * targetHeightDifference) + (errorAccDistance * errorAccDistance));
    double nextHypotDistance = Math.sqrt(squared);

    // ill refactor this later.
    double hypotIndex0 = Math.floor(cachedHypotDistances.get(0));
    double hypotIndex1 = Math.floor(cachedHypotDistances.get(1));
    double hypotIndex2 = Math.floor(cachedHypotDistances.get(2));
    double hypotIndex3 = Math.floor(cachedHypotDistances.get(3));
    double hypotIndex4 = Math.floor(cachedHypotDistances.get(4));
    double hypotIndex5 = Math.floor(cachedHypotDistances.get(5));
    double hypotIndex6 = Math.floor(cachedHypotDistances.get(6));
    double hypotIndex7 = Math.floor(cachedHypotDistances.get(7));
    double hypotIndex8 = Math.floor(cachedHypotDistances.get(8));
    double hypotIndex9 = Math.floor(cachedHypotDistances.get(9));
    double hypotIndex10 = Math.floor(cachedHypotDistances.get(10));

    double hypotenuseDistance = (hypotIndex0 + hypotIndex1 + hypotIndex2 + hypotIndex3 + hypotIndex4 + hypotIndex5
        + hypotIndex6 + hypotIndex7 + hypotIndex8 + hypotIndex9 + hypotIndex10) / 11;

    cachedHypotDistances.add(nextHypotDistance);
    cachedHypotDistances.remove(0);

    return hypotenuseDistance;
  }

  public double findDistance() {
    // from documentation, the distance can be found using a fixed camera angle
    // distance = (height2 - height1) / tan(angle1 + angle2)
    // height1 is limelight elevation, height2 is target height
    // angle1 is limelight angle, angle2 is target angle
    // What we need: limelight angle on the robot, distance from center of limelight
    // lens to ground,
    // distance from height of the target to the floor

    NetworkTable table = NetworkTableInstance
        .getDefault()
        .getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");

    double offsetX = tx.getDouble(0.0) + horzontalOffset.get();

    // double changeAngle = SmartDashboard.getNumber("LIMELIGHT SET ANGLE: ",
    // limelightMountedAngle);

    double verticalOffsetAngle = ty.getDouble(0.0); // angle calculated by the limelight.

    double angleToGoalDeg = (limelightMountedAngle + verticalOffsetAngle);
    double angleToGoalRad = angleToGoalDeg * (3.14159 / 180.0);
    double error = 0.612649568;

    double distance = ((reflectiveTapeHeight - limelightLensHeight) /
        (Math.tan(angleToGoalRad)));

    // The following statements were used as ways to account for error that the
    // limelight gives from distance
    // There are a bunch of different statements to be more prescise at every
    // distance
    if (32.905 <= distance && distance <= 37.6) {
      error = 0.685521;
    }
    if (37.7 <= distance && distance <= 43.4) {
      error = 0.64395;
    }
    if (43.5 <= distance && distance <= 48.5) {
      error = 0.6303;
    }
    if (48.6 <= distance && distance <= 54.5) {
      error = 0.641166666666667;
    }
    if (54.6 <= distance && distance <= 62.5) {
      error = 0.609114583333333;
    }
    if (62.6 <= distance && distance <= 69.40) {
      error = 0.610555555555556;
    }
    if (69.41 <= distance && distance <= 76.43) {
      error = 0.611683333333333;
    }
    if (79.44 <= distance && distance <= 89.4) {
      error = 0.674242424242424;
    }
    if (89.41 <= distance && distance <= 91.2) {
      error = 0.623541666666667;
    }
    if (91.21 <= distance && distance <= 98.416) {
      error = 0.589606456;
    }
    if (98.417 <= distance && distance <= 162.5) {
      error = 0.555793264;
    }

    errorAccDistance = (distance / error);

    if (58.4 <= errorAccDistance && errorAccDistance <= 73.2) {
      errorAccDistance = errorAccDistance * 0.926887142;
    }
    if (58.4 <= errorAccDistance && errorAccDistance <= 73.2) {
      errorAccDistance = errorAccDistance * 0.926887142;
    }
    if (87.41 <= errorAccDistance && errorAccDistance <= 98.3) {
      errorAccDistance = errorAccDistance * 0.952870388;
    }
    if (162.51 <= errorAccDistance && errorAccDistance <= 188.4) {
      errorAccDistance = errorAccDistance * 0.908051108;
    }
    if (188.41 <= errorAccDistance && errorAccDistance <= 210.2) {
      errorAccDistance = errorAccDistance * 0.94147961;
    } else {
      errorAccDistance = distance / error;
    }

    errorAccDistance = errorAccDistance * 0.73;

    double height = (reflectiveTapeHeight - limelightLensHeight);

    double squared = ((height * height) + (errorAccDistance * errorAccDistance));

    double hypotenuseDistance = Math.sqrt(squared);

    SmartDashboard.putNumber("Error Acc Distance", errorAccDistance);

    return errorAccDistance;
  }

  @Deprecated
  public Supplier<Double> getDistanceSupplier() {
    return this.distanceSupplier;
  }

  @Deprecated
  public Supplier<Double> getAngleOffsetSupplier() {
    return this.horzontalOffset;
  }

  @Deprecated
  public Supplier<Double> getAngleSupplier() {
    return this.angleSupplier;
  }

  @Deprecated
  public Supplier<Double> getHypotDistance() {
    return this.hypotDistanceSupplier;
  }

  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance
        .getDefault()
        .getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");

    double offsetX = tx.getDouble(0.0);
    double offsetY = ty.getDouble(0.0);
    double objectArea = ta.getDouble(0.0);
    double robotSkew = ts.getDouble(0.0);

    SmartDashboard.putNumber("Object Offset X: ", offsetX);
    SmartDashboard.putNumber("Object Offset Y: ", offsetY);
    SmartDashboard.putNumber("Limelight Area: ", objectArea);
    SmartDashboard.putNumber("Limelight Skew: ", robotSkew);
    SmartDashboard.putBoolean("Limelight On: ", limelightLedOn);
    SmartDashboard.putNumber("LIMELIGHT SET ANGLE: ", limelightMountedAngle);

    SmartDashboard.getNumber("limelight angle offset", horzontalOffset.get());
    if (limelightLedOn == true) {
      findDistance();
    }
  }
}
