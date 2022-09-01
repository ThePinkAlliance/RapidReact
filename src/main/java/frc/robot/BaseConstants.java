package frc.robot;

public class BaseConstants {

  public static final double MAX_SPEED = 1;

  public static Gains targetTrackerGains = new Gains(6.0, 0.0, 0.0); // Tuned at SLF w/ BASE_TRACKER_LIMIT
  public static Gains navigateTurnGains = new Gains(6.0, 0.0, 0.0); // Tuned at SLF w/ BASE_TRACKER_LIMIT
  public static Gains navigateDriveGains = new Gains(1.0, 0.0, 0.0);
}
