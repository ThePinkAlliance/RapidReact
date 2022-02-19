// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Color;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TempTower;

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

  private final Joystick gamepad_base = new Joystick(0);
  private final Base m_base = new Base();
  // private final Shooter m_shooter = new Shooter();
  // private final TempTower tower = new TempTower();

  double kP_X = 0;
  double kD_X = 0;
  double kI_X = 0;

  double kP_Y = 0;
  double kD_Y = 0;
  double kI_Y = 0;

  double kP_T = 0;
  double kI_T = 0;
  double kD_T = 0;

  Trajectory trajectory = new Trajectory();
  ShuffleboardTab driverDashboard = Shuffleboard.getTab("Dashboard");
  SendableChooser<SelectableTrajectory> selectedPath = new SendableChooser<SelectableTrajectory>();

  public SwerveControllerCommand swerveController = new SwerveControllerCommand(
    trajectory,
    m_base::getPose,
    m_base.kinematics,
    // these values are filler's
    // this is the x axis PID Controller
    new PIDController(
      SmartDashboard.getNumber("kP-X", kP_X),
      SmartDashboard.getNumber("kI-X", kI_X),
      SmartDashboard.getNumber("kD-X", kP_X)
    ),
    // this is the y axis PID Contrller
    new PIDController(
      SmartDashboard.getNumber("kP-Y", kP_Y),
      SmartDashboard.getNumber("kI-Y", kI_Y),
      SmartDashboard.getNumber("kD-Y", kD_Y)
    ),
    new ProfiledPIDController(
      SmartDashboard.getNumber("theta-P", kP_T),
      SmartDashboard.getNumber("theta-I", kI_T),
      SmartDashboard.getNumber("theta-D", kD_T),
      new TrapezoidProfile.Constraints(
        Constants.Base.MAX_VELOCITY_METERS_PER_SECOND,
        Constants.Base.MAX_ACCELERATION_METERS_PER_SECOND
      )
    ),
    states -> {
      m_base.setStates(states);
    },
    m_base
  );

  private final SelectableTrajectory leaveBlueLeft = new SelectableTrajectory(
    "Leave Blue Left",
    "output/LeaveBlueLeft.wpilib.json"
  );

  private final SelectableTrajectory straight = new SelectableTrajectory(
    "Straight",
    "output/Straight.wpilib.json"
  );

  /**
   * This contains all the trajectories that can be selected from the dashboard.
   */
  private final SelectableTrajectory[] trajectories = {
    leaveBlueLeft,
    straight,
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putNumber("kP-X", kP_X);
    SmartDashboard.putNumber("kD-X", kD_X);

    SmartDashboard.putNumber("kP-Y", kP_Y);
    SmartDashboard.putNumber("kD-Y", kD_Y);

    for (SelectableTrajectory t : trajectories) {
      if (t.location == straight.location) {
        selectedPath.setDefaultOption(t.name, t);
      } else {
        selectedPath.addOption(t.name, t);
      }
    }
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
        new Drive(
          m_base,
          () -> gamepad_base.getRawAxis(0),
          () -> gamepad_base.getRawAxis(1),
          () -> gamepad_base.getRawAxis(4),
          this.gamepad_base
        )
      );
    // this.m_shooter.setDefaultCommand(
    //     new Shoot(m_shooter, () -> gamepad_base.getRawAxis(2))
    //   );
    new JoystickButton(gamepad_base, 1).whenPressed(m_base::zeroGyro);
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

    // set the initial pose of the robot to the starting pose of the trajectory
    m_base.resetOdometry(trajectory.getInitialPose());

    return selectedPath.getSelected().getDefualtCommand();
  }
}
