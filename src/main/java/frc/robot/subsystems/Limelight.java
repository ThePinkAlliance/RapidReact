// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Debug;

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

  private final NetworkTable table = NetworkTableInstance
      .getDefault()
      .getTable("limelight");

  SlewRateLimiter limiter = new SlewRateLimiter(20);

  double errorAccDistance = 0;
  double limelightMountedAngle = 51.5; // 50 // this can change a static number though once we have found it

  /** Creates a new Limelight. */
  public Limelight() {
    configureLimelight(LimelightLedMode.FORCE_OFF);
  }

  public void configureLimelight(LimelightLedMode mode) {
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
    configureLimelight(mode); // Off or On
  }

  public boolean isTarget() {
    NetworkTableEntry tv = table.getEntry("tv");
    double availableTargets = tv.getDouble(0.0);

    return availableTargets == 1;
  }

  public double getOffset() {
    NetworkTableEntry tx = table.getEntry("tx");
    double offsetX = tx.getDouble(0.0);

    return offsetX;
  }

  public double calculateUnmodifiedDistance() {
    NetworkTableEntry ty = table.getEntry("ty");

    double verticalOffsetAngle = ty.getDouble(0.0); // angle calculated by the limelight.

    double angleToGoalDeg = (limelightMountedAngle + verticalOffsetAngle);

    /*
     * Converts the estimated angle from the target in degress to radians.
     */
    double angleToGoalRad = angleToGoalDeg * (3.14159 / 180.0);

    /*
     * Calculates the distance using the known target height and limelight height
     * then subtracting the difference from them and dividing them by the tangant of
     * the estimated angle from the target in radians.
     */
    return ((reflectiveTapeHeight - limelightLensHeight) /
        (Math.tan(angleToGoalRad)));
  }

  public double calculateAccountedDistance() {
    /*
     * from documentation, the distance can be found using a fixed camera angle
     * distance = (height2 - height1) / tan(angle1 + angle2)
     * height1 is limelight elevation, height2 is target height
     * angle1 is limelight angle, angle2 is target angle
     * What we need: limelight angle on the robot, distance from center of limelight
     * lens to ground, distance from height of the target to the floor
     */

    NetworkTableEntry ty = table.getEntry("ty");

    /*
     * angle calculated by the limelight, which needs to be corrected as its not
     * super accurate.
     */
    double verticalOffsetAngle = ty.getDouble(0.0);

    double angleToGoalDeg = (limelightMountedAngle + verticalOffsetAngle);

    /*
     * Converts the estimated angle from the target in degress to radians.
     */
    double angleToGoalRad = angleToGoalDeg * (3.14159 / 180.0);

    /*
     * Calculates the distance using the known target height and limelight height
     * then subtracting the difference from them and dividing them by the tangant of
     * the estimated angle from the target in radians.
     */
    double hypot = ((reflectiveTapeHeight - limelightLensHeight) /
        (Math.tan(angleToGoalRad)));

    /*
     * Interpolates between the two closest vectors to the hypotenuse.
     */
    double distance = Constants.limelightInterpolationTable.interp(hypot);

    Debug.putNumber("interp-distance", distance);

    return distance;
  }

  public double calculateDistanceHypot() {
    double errorAccDistance = calculateAccountedDistance();
    double squared = ((targetHeightDifference * targetHeightDifference) + (errorAccDistance * errorAccDistance));
    double nextHypotDistance = Math.sqrt(squared);

    /*
     * The slew rate limiter will limit the rate of change of the distance.
     * 
     * NOTE: This is only a temporary solution we should adjust speckel rejection to
     * prevent noise from entering the stream.
     */
    // double hypotenuseDistance = limiter.calculate(nextHypotDistance);

    return nextHypotDistance;
  }

  /**
   * 
   * 
   * @deprecated This method is from bayou and is using a inefficent system to
   *             correct the
   *             distance being reported from the limelight please don't use this
   *             in any new
   *             commands.
   */
  @Deprecated
  public double findDistance() {
    // from documentation, the distance can be found using a fixed camera angle
    // distance = (height2 - height1) / tan(angle1 + angle2)
    // height1 is limelight elevation, height2 is target height
    // angle1 is limelight angle, angle2 is target angle
    // What we need: limelight angle on the robot, distance from center of limelight
    // lens to ground,
    // distance from height of the target to the floor

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

  /**
   * 
   * 
   * @deprecated This method uses the now deprecated method findDistance please
   *             don't use this method in any new systems.
   */
  @Deprecated
  public Supplier<Double> getDistanceSupplier() {
    return this.distanceSupplier;
  }

  /**
   * 
   * 
   * @deprecated This method uses the now deprecated method findDistance please
   *             don't use this method in any new systems.
   */
  @Deprecated
  public Supplier<Double> getAngleOffsetSupplier() {
    return this.horzontalOffset;
  }

  /**
   * 
   * 
   * @deprecated This method uses the now deprecated method findDistance please
   *             don't use this method in any new systems.
   */
  @Deprecated
  public Supplier<Double> getAngleSupplier() {
    return this.angleSupplier;
  }

  /**
   * 
   * 
   * @deprecated This method uses the now deprecated method findDistance please
   *             don't use this method in any new systems.
   */
  @Deprecated
  public Supplier<Double> getHypotDistance() {
    return this.hypotDistanceSupplier;
  }

  @Override
  public void periodic() {

  }
}
