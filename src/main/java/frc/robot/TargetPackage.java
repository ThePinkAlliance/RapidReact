package frc.robot;

import frc.robot.subsystems.Hood;

public class TargetPackage {
	public final double Kp;
	public final double Kf;
	public final double hoodPosition;
	public final double rpm;
	
	public TargetPackage(double Kp, double kF, double hoodPosition, double rpm){
		this.Kp = Kp;
	    this.Kf = kF;
		this.rpm = rpm;
		this.hoodPosition = hoodPosition;
	}
}