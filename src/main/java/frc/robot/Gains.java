package frc.robot;

public class Gains {

  public final double kP;
  public final double kI;
  public final double kD;

  public Gains(double _kP, double _kI, double _kD) {
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
}
