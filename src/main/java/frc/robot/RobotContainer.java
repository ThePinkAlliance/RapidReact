// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Base;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick gamepad_base = new Joystick(0);
  private final Base m_base = new Base();

  private final SelectableTrajectory leaveBlueLeft = new SelectableTrajectory("Leave Blue Left",
      "output/LeaveBlueLeft.wpilib.json");

  Trajectory trajectory = new Trajectory();
  ShuffleboardTab driverDashboard = Shuffleboard.getTab("dashboard");
  SendableChooser<SelectableTrajectory> selectedPath = new SendableChooser<SelectableTrajectory>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    selectedPath.setDefaultOption(leaveBlueLeft.name, leaveBlueLeft);

    driverDashboard.add(selectedPath);

    // for now select leave blue 1 for testing
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.m_base.setDefaultCommand(
        new Drive(m_base, () -> gamepad_base.getRawAxis(0),
            () -> gamepad_base.getRawAxis(1), () -> gamepad_base.getRawAxis(2)));
  }

  public void selectTrajectory(SelectableTrajectory selectableTrajectory) {
    this.trajectory = selectableTrajectory.trajectory;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // set the current trajectory to execute
    selectTrajectory(selectedPath.getSelected());

    m_base.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));

    return new SwerveControllerCommand(
        trajectory,
        m_base::getPose,
        m_base.kinematics,
        // these values are filler's
        // this is the x axis PID Controller
        new PIDController(0.4, 0, 0),
        // this is the y axis PID Contrller
        new PIDController(0.4, 0, 0),
        new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(Constants.MAX_VELOCITY_METERS_PER_SECOND,
                Constants.MAX_ACCELERATION_METERS_PER_SECOND)),
        (states) -> {
          m_base.setStates(states);
        },
        m_base);
  }
}
