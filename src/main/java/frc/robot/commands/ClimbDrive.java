// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Climbers;

public class ClimbDrive extends CommandBase {

  Base base;
  Climbers climbers;
  double power;
  Timer timer;

  double align_kP = 3.7;
  double align_kI = 0.5;
  double align_kD = 0.003;

  boolean bBackwards = false;

  /**
   * kP:
   * kI:
   * kD: keep kD low otherwise your system could become unstable
   */

  PIDController alignController = new PIDController(
    align_kP,
    align_kI,
    align_kD
  );

  // 0.122807
  double reduction = SdsModuleConfigurations.MK4I_L1.getDriveReduction();

  double targetAngle = 0;
  double watchDogTime = 3;
  
  /** Creates a new DriveStraight. */
  public ClimbDrive(Base base, Climbers climbers, double targetAngle, double power, boolean backwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.climbers = climbers;
    
    this.targetAngle = targetAngle;
    timer = new Timer();
    

    addRequirements(base, climbers);
  }

 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.drive(new ChassisSpeeds());
    alignController.reset();
 
    base.zeroGyro();
    base.resetDriveMotors();

    alignController.enableContinuousInput(-180, 180);

    alignController.setTolerance(0.5, 1.0);

    timer.reset();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = base.getSensorYaw();
    double angle_error = alignController.calculate(currentAngle, targetAngle);
    double theta_power =
      (angle_error / -180) * Base.MAX_VELOCITY_METERS_PER_SECOND;

    power = power * Base.MAX_VELOCITY_METERS_PER_SECOND;
    
    if (bBackwards)
       power *= -1;
    ChassisSpeeds speeds = new ChassisSpeeds(power, 0, theta_power);
    if (this.climbers.shortClimberModule.contactedRightPole() && this.climbers.shortClimberModule.contactedLeftPole())
       this.climbers.closeShortArms();

    base.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(new ChassisSpeeds(0, 0, 0));
    base.resetDriveMotors();

   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean value = false;
     if (climbers.shortClimberModule.getSolenoidState() || timer.get() > watchDogTime)
       value = true;
     else
       value = false;
    System.out.println("ClimbDrive:  isFinished: " + value + " / " + timer.get());
    return value;
  }
}
