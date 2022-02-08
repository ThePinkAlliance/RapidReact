package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

// this will be used to control the angle of the climber module
public class AngleTalon extends TalonFX {

  public AngleTalon(int deviceNumber) {
    super(deviceNumber);

    /* Config the peak and nominal outputs, 12V means full */
    this.configNominalOutputForward(0, AngleConstants.kTimeoutMs);
    this.configNominalOutputReverse(0, AngleConstants.kTimeoutMs);
    this.configPeakOutputForward(1, AngleConstants.kTimeoutMs);
    this.configPeakOutputReverse(-1, AngleConstants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    this.configAllowableClosedloopError(0, AngleConstants.kPIDLoopIdx, AngleConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    this.config_kF(AngleConstants.kPIDLoopIdx, AngleConstants.kGains.kF, AngleConstants.kTimeoutMs);
    this.config_kP(AngleConstants.kPIDLoopIdx, AngleConstants.kGains.kP, AngleConstants.kTimeoutMs);
    this.config_kI(AngleConstants.kPIDLoopIdx, AngleConstants.kGains.kI, AngleConstants.kTimeoutMs);
    this.config_kD(AngleConstants.kPIDLoopIdx, AngleConstants.kGains.kD, AngleConstants.kTimeoutMs);
  }

}

class AngleConstants {
  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
   * or 3. Only the first two (0,1) are visible in web-based configuration.
   */
  public static final int kSlotIdx = 0;

  /**
   * Talon FX supports multiple (cascaded) PID loops. For now we just want the
   * primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kSensorPhase = true;

  /**
   * Choose based on what direction you want to be positive, this does not affect
   * motor invert.
   */
  public static boolean kMotorInvert = false;

  /**
   * Number of joystick buttons to poll. 10 means buttons[1,9] are polled, which
   * is actually 9 buttons.
   */
  public final static int kNumButtonsPlusOne = 10;

  /**
   * How many sensor units per rotation. Using Talon FX Integrated Sensor.
   * 
   * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
   */
  public final static int kSensorUnitsPerRotation = 2048;

  /**
   * Number of rotations to drive when performing Distance Closed Loop
   */
  public final static double kRotationsToTravel = 6;

  /**
   * Set to zero to skip waiting for confirmation. Set to nonzero to wait and
   * report to DS if action fails.
   */
  public final static int kTimeoutMs = 30;

  /**
   * Motor neutral dead-band, set to the minimum 0.1%.
   */
  public final static double kNeutralDeadband = 0.001;

  /**
   * Empirically measure what the difference between encoders per 360' Drive the
   * robot in clockwise rotations and measure the units per rotation. Drive the
   * robot in counter clockwise rotations and measure the units per rotation. Take
   * the average of the two.
   */
  public final static int kEncoderUnitsPerRotation = 2048;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 6800 represents
   * Velocity units at 100% output Not all set of Gains are used in this project
   * and may be removed as desired.
   * 
   * kP kI kD kF Iz PeakOut
   */
  public final static Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
  public final static Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
  public final static Gains kGains_Velocit = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
  public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);

  /**
   * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki,
   * kd, kf, izone, peak output);
   */
  public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0, 100, 0.50);

  /** ---- Flat constants, you should not need to change these ---- */
  /*
   * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
   * can have up to 2 devices assigned remotely to a talon/victor]
   */
  public final static int REMOTE_0 = 0;
  public final static int REMOTE_1 = 1;
  /*
   * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
   * is auxiliary
   */
  public final static int PID_PRIMARY = 0;
  public final static int PID_TURN = 1;
}