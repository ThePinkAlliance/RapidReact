package frc.robot;

public class ShooterConstants {

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from
   * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
   * configuration.
   */
  public static final int kSlotIdx = 0;

  /**
   * Talon FX supports multiple (cascaded) PID loops. For
   * now we just want the primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int kTimeoutMs = 30;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kSensorPhase = true;

  /**
   * Choose based on what direction you want to be positive,
   * this does not affect motor invert.
   */
  public static boolean kMotorInvert = false;

  /**
   * Gains used in Positon Closed Loop, to be adjusted accordingly
   * Gains(kp, ki, kd, kf, izone, peak output);
   */
  public static final ShooterGains kGains = new ShooterGains(
    0.2,
    0.0,
    0.0,
    0.046,
    0,
    1.0
  );

  public static final ShooterGains kGainsTarmac = new ShooterGains(
    0.4,
    0,
    0,
    0.047,
    0,
    0
  );

  public static final double SHOOTER_POWER_HUB_HIGH = 2450;
  public static final double SHOOTER_POWER_TARMAC_HIGH = 2730;
  public static final double SHOOTER_POWER_THREE_BALL = 2730;
  public static final double SHOOTER_POWER_AUTO_TWO_BALL = 2750;
  public static final double SHOOTER_Kp_AUTO_TWO_BALL = 0.4;
  public static final double SHOOTER_FF_AUTO_TWO_BALL = 0.047;
  public static final double SHOOTER_POWER_HUB_LOW = 1800;

  public static final double SHOOTER_Kp_AUTO_THREE_BALL = 0.4;
  public static final double SHOOTER_FF_AUTO_THREE_BALL = 0.047;

  // public static final ShooterGains kGains = new ShooterGains(0.2, 0.0, 0.0, 0.046, 0, 1.0);
  public static double ALLOWABLE_CLOSELOOP_ERROR = 0;
}
