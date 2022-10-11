
import edu.wpi.first.math.util.Units;
import frc.robot.HoodConstants;
import frc.robot.TargetPackageFactory;
import frc.robot.subsystems.Hood;
import org.junit.Test;

public class ProjectileCalculationsTest {
  final double g = Math.pow(9.81, 2);
  final double targetHeight = 2.64; // meters
  final double targetDistance = 3.302; // meters
  final double targetAngle = Units.radiansToDegrees(targetHeight / targetDistance);

  final double shooterWheelDiameter = 4.063;
  final double shooterWheelRadius = shooterWheelDiameter / 2;
  final double shooterWheelCircmfrence = 2 * shooterWheelRadius * Math.PI;

  final double desiredApproachAngle = -59;

  /**
   * This converts rpm to m/s using the shooter wheels circumference.
   */
  private double convertRpm(double rpm) {
    return Units.inchesToMeters(shooterWheelCircmfrence) * (rpm / 60);
  }

  @Test
  public void HitTargetCheck() {
    double rpm = TargetPackageFactory.getCustomPackage(Units.metersToInches(targetDistance)).rpm;
    double hoodPosition = TargetPackageFactory.getCustomPackage(Units.metersToInches(targetDistance)).hoodPosition;

    double angle = Units.radiansToDegrees((Math.atan((Math.tan(desiredApproachAngle) * targetDistance - 2
        * targetHeight) / (-targetDistance))));
    double velocity = Math.sqrt(
        -(9.8
            * Math.pow(targetDistance, 2) * (1 + (Math.tan(desiredApproachAngle) * Math.tan(desiredApproachAngle))))
            / (2 * targetHeight - 2 * targetDistance * Math.tan(desiredApproachAngle)));
    double eVelocity = convertRpm(rpm);

    double estAngle = Units.radiansToDegrees((Units.degreesToRadians(Hood.getHoodAngle(hoodPosition))));

    System.out.println("Calculated Angle: " + angle);
    System.out.println("Target Angle: " + estAngle);
    System.out.println("Calculated Velocity: " + velocity);
    System.out.println("Target Velocity: " + eVelocity);
    System.out.println("Hood Angle: " + (Hood.getHoodAngle(HoodConstants.HUB_LOW_SHOT_COUNT)));

    // assertEquals(velocity, eVelocity, 0);
    // assertEquals(velocity, convertRpm(rpm), 0.1);
  }
}
