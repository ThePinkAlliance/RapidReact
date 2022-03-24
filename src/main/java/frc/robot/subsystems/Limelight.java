// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ADDRESS FOR THE LIMELIGHT FEED: http://limelight.local:5801/

public class Limelight extends SubsystemBase {

  private boolean limelightLedOn = false;

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
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");

    double offsetX = tx.getDouble(0.0);
    double objectArea = ta.getDouble(0.0);
    double robotSkew = ts.getDouble(0.0);

    double limelightMountedAngle = 45; //this can change a static number though once we have found it
    double limelightLensHeight = 24; //this can change (in) will be static, should NEVER change
    double reflectiveTapeHeight = 102.375; //this is static (in) to CENTER of reflective tape
    double verticalOffsetAngle = ty.getDouble(0.0); //angle calculated by the limelight.

    double angleToGoalDeg = (limelightMountedAngle + verticalOffsetAngle);
    double angleToGoalRad = angleToGoalDeg * (Math.PI / 180.0);

    double distance =
      (reflectiveTapeHeight - limelightLensHeight) / Math.tan(angleToGoalRad);
    SmartDashboard.putNumber("Distance: ", distance);

    SmartDashboard.putNumber("Object Offset X: ", offsetX);
    SmartDashboard.putNumber("Object Offset Y: ", verticalOffsetAngle);
    SmartDashboard.putNumber("Limelight Area: ", objectArea);
    SmartDashboard.putNumber("Limelight Skew: ", robotSkew);
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
    if (limelightLedOn == true) {
      getDistance();
    }
  }
}
