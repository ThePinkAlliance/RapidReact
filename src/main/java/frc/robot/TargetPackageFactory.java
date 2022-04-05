package frc.robot;


public class TargetPackageFactory {
	
private static final double RPM_CONSTANT_1 = 7.5; //8.456;
private static final double RPM_CONSTANT_2 = 2000;//2118;
private static final double HOOD_POSITION_CONSTANT_1 =  -244.8;
private static final double HOOD_POSITION_CONSTANT_2 = 37634;
private static final double HOOD_POSITION_MAX = -78000;
private static final double HOOD_POSITION_MIN = -200;

	private static TargetPackage lowPackage = new TargetPackage(
    ShooterConstants.kGains.kP,
    ShooterConstants.kGains.kF,
    HoodConstants.HUB_LOW_SHOT_COUNT,
    ShooterConstants.SHOOTER_POWER_HUB_LOW
  );

  private static TargetPackage highPackage = new TargetPackage(
    ShooterConstants.kGains.kP,
    ShooterConstants.kGains.kF,
    HoodConstants.HUB_SHOT_TICK_COUNT,
    ShooterConstants.SHOOTER_POWER_HUB_HIGH
  );

  private static TargetPackage tarmacPackage = new TargetPackage(
    ShooterConstants.kGainsTarmac.kP,
    ShooterConstants.kGainsTarmac.kF,
    HoodConstants.TARMAC_SHOT_TICK_COUNT,
    ShooterConstants.SHOOTER_POWER_TARMAC_HIGH
  );

  private static TargetPackage twoBallPackage = new TargetPackage(
    ShooterConstants.SHOOTER_Kp_AUTO_TWO_BALL,
    ShooterConstants.SHOOTER_FF_AUTO_TWO_BALL,
    HoodConstants.AUTO_SHOT_TWOBALL_TICK_COUNT,
    ShooterConstants.SHOOTER_POWER_TARMAC_HIGH + 50
  );

  private static TargetPackage threeBallPackage = new TargetPackage(
      ShooterConstants.SHOOTER_Kp_AUTO_THREE_BALL,
      ShooterConstants.SHOOTER_FF_AUTO_THREE_BALL,
      HoodConstants.AUTO_SHOT_THREEBALL_TICK_COUNT,
      ShooterConstants.SHOOTER_POWER_THREE_BALL
    );

  //Custom:  new it up here so that its not newed up with every cycle
  public static TargetPackage customPackage = new TargetPackage(0, 0, 0, 0);

	public TargetPackageFactory(){}

	public static TargetPackage getLowHubPackage() {
		return lowPackage;
	}

	public static TargetPackage getHighHubPackage() {
		return highPackage;
	}

	public static TargetPackage getTarmacPackage() {
		return tarmacPackage;
	}

	public static TargetPackage getTwoBallPackage() {
		return twoBallPackage;
	}

  public static TargetPackage getThreeBallPackage() {
		return threeBallPackage;
	}

  public static TargetPackage getCustomPackage(double distance) {

    //SLF: linear formula based on testing with limelight and shooting from various positions
    customPackage.rpm = (RPM_CONSTANT_1 * distance) + RPM_CONSTANT_2;
    customPackage.hoodPosition = (HOOD_POSITION_CONSTANT_1 * distance) - HOOD_POSITION_CONSTANT_2;
    if (customPackage.hoodPosition <= HOOD_POSITION_MAX)
       customPackage.hoodPosition = HOOD_POSITION_MAX;
    else if (customPackage.hoodPosition > HOOD_POSITION_MIN)
       customPackage.hoodPosition = HOOD_POSITION_MIN;
    customPackage.Kf = customPackage.rpm / ShooterConstants.SHOOTER_MAGIC_NUMBER; 
    customPackage.Kp = ShooterConstants.kGainsRange.kP;

    return customPackage;
  }
}