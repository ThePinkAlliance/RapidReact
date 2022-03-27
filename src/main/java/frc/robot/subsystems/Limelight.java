// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

// ADDRESS FOR THE LIMELIGHT FEED: http://limelight.local:5801/

public class Limelight extends SubsystemBase {

  private boolean limelightLedOn = false;

  private double horzontalOffset = 0;
  private Supplier<Double> distanceSupplier = () -> 0.0;
  private Supplier<Double> angleSupplier = () -> 0.0;

  /** Creates a new Limelight. */
  public Limelight() {
    initLimelight(LimelightLedMode.FORCE_OFF);
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
    initLimelight(mode); //Off or On
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
      targets = false; //not necessary but for safety
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

  public void getDistance() {
    //from documentation, the distance can be found using a fixed camera angle
    //distance = (height2 - height1) / tan(angle1 + angle2)
    //height1 is limelight elevation, height2 is target height
    //angle1 is limelight angle, angle2 is target angle
    //What we need: limelight angle on the robot, distance from center of limelight lens to ground,
    //distance from height of the target to the floor

    NetworkTable table = NetworkTableInstance
      .getDefault()
      .getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");

    double offsetX = tx.getDouble(0.0) + horzontalOffset;

    double limelightMountedAngle = 50; //this can change a static number though once we have found it
    double limelightLensHeight = 33.5; //this can change (in) will be static, should NEVER change
    double reflectiveTapeHeight = 102.375; //this is static (in) to CENTER of reflective tape
    double verticalOffsetAngle = ty.getDouble(0.0); //angle calculated by the limelight.

    double angleToGoalDeg = (limelightMountedAngle + verticalOffsetAngle);
    double angleToGoalRad = angleToGoalDeg * (3.14159 / 180.0);
    double error = 0.612649568;

    double distance =
      (
        (reflectiveTapeHeight - limelightLensHeight) /
        (Math.tan(angleToGoalRad))
      );

    //The following statements were used as ways to account for error that the limelight gives from distance
    //There are a bunch of different statements to be more prescise at every distance
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
    if (98.417 <= distance) {
      error = 0.555793264;
    } else {
      error = 0.612649568;
    }

    double errorAccDistance = (distance / error);
    double distanceInFeet = errorAccDistance / 12;

    this.distanceSupplier = () -> distanceInFeet;
    this.angleSupplier = () -> offsetX;

    SmartDashboard.putNumber("Distance: ", errorAccDistance);
    SmartDashboard.putNumber("Distance in feet: ", distanceInFeet);
    SmartDashboard.putNumber("Object Offset X: ", offsetX);
    SmartDashboard.putNumber("Object Offset Y: ", verticalOffsetAngle);
  }

  public Supplier<Double> getDistanceSupplier() {
    return this.distanceSupplier;
  }

  public Supplier<Double> getAngleSupplier() {
    return this.angleSupplier;
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

    SmartDashboard.getNumber("limelight angle offset", horzontalOffset);

    if (limelightLedOn == true) {
      getDistance();
    }
  }
}
