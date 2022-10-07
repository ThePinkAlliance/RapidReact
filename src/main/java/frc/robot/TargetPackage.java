package frc.robot;

public class TargetPackage {
	public double Kp;
	public double Kf;
	public double hoodPosition;
	public double rpm;

	public TargetPackage(double Kp, double kF, double hoodPosition, double rpm) {
		this.Kp = Kp;
		this.Kf = kF;
		this.rpm = rpm;
		this.hoodPosition = hoodPosition;
	}
}