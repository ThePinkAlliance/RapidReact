// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotHealth extends SubsystemBase {

  private PowerDistribution pdp;

  /** Creates a new RobotHealth. */
  public RobotHealth(PowerDistribution pdp) {
    this.pdp = pdp;
  }

  public void reportError(String message) {
    DriverStation.reportError(message, false);
  }

  public void reportError(String message, double amps, double volts) {
    String fullMessage =
      message + " , total amps: " + amps + ", total volts: " + volts;

    DriverStation.reportError(fullMessage, false);
  }

  public void reportWarning(String message) {
    DriverStation.reportWarning(message, false);
  }

  public void reportErrorStacktrace(String message) {
    DriverStation.reportError(message, true);
  }

  public void reportWarningStacktrace(String message) {
    DriverStation.reportWarning(message, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (this.pdp != null) {
      double totalAmps = pdp.getTotalCurrent();
      double totalVolts = pdp.getVoltage();

      if (pdp.getStickyFaults().HasReset) {
        this.reportError("PDP has reset", totalAmps, totalVolts);
      }

      if (pdp.getStickyFaults().Brownout) {
        this.reportError("PDP is experiencing brownout", totalAmps, totalVolts);
      }

      if (pdp.getStickyFaults().Channel11BreakerFault) {
        this.reportError("Shooter PDP Breaker Fault", totalAmps, totalVolts);
      }
    }
  }
}
