// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
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
import frc.robot.commands.BasicAuto;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LeaveBlueLeft;
import frc.robot.commands.RotateLeft;
import frc.robot.commands.Shoot;
import frc.robot.commands.StraightAuto;
import frc.robot.commands.TurretRotate;
import frc.robot.commands.TurretRotate;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightLedMode;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TempTower;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Limelight;

import java.security.Principal;
import java.util.List;

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
  private final Limelight m_limelight = new Limelight();
  // private final Turret m_turret = new Turret();
  // private final Shooter m_shooter = new Shooter();
  // private final TempTower tower = new TempTower();

  // these values are filler's
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

  TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    Base.MAX_VELOCITY_METERS_PER_SECOND,
    Base.MAX_ACCELERATION_METERS_PER_SECOND
  )
  .setKinematics(m_base.kinematics);

  // this trajectory will drive 9.10 feet
  // private Trajectory goStraight = TrajectoryGenerator.generateTrajectory(
  //   new Pose2d(0, 0, new Rotation2d(0)),
  //   List,
  //   new Pose2d(3, 0, new Rotation2d(0)),
  //   trajectoryConfig
  // );

  private Trajectory goStraight = TrajectoryGenerator.generateTrajectory(
    List.of(
      new Pose2d(0, 0, new Rotation2d(0)),
      new Pose2d(3, 0, new Rotation2d(0))
    ),
    trajectoryConfig
  );

  private final SelectableTrajectory leaveBlueLeft = new SelectableTrajectory(
    "Leave Blue Left",
    new LeaveBlueLeft(m_base)
  );

  /**
   * This contains all the trajectories that can be selected from the dashboard.
   */
  private final SelectableTrajectory[] trajectories = { leaveBlueLeft };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putNumber(
      Turret.TURRET_NAME + " power",
      Turret.TURRET_DEFAULT_POWER
    );

    for (SelectableTrajectory t : trajectories) {
      if (t.location == leaveBlueLeft.name) {
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
    new JoystickButton(gamepad_base, Constants.JOYSTICK_BUTTON_A)
    .whenPressed(m_base::zeroGyro);
    // new JoystickButton(gamepad_base, Constants.JOYSTICK_BUTTON_X)
    // .whenPressed(
    //     new TurretRotate(
    //       m_turret,
    //       gamepad_base,
    //       SmartDashboard.getNumber(
    //         Turret.TURRET_NAME + " power",
    //         Turret.TURRET_DEFAULT_POWER
    //       )
    //     )
    //   );
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
    // // set the current trajectory to execute
    // selectTrajectory(selectedPath.getSelected());

    // set the initial pose of the robot to the starting pose of the trajectory
    // m_base.resetOdometry(trajectory.getInitialPose());

    if (selectedPath.getSelected().getDefualtCommand() == null) {
      selectedPath.setDefaultOption("Leave Blue Left", leaveBlueLeft);
    }

    return selectedPath.getSelected().getDefualtCommand();
  }

  public void enableLimelight() {
    if (m_limelight != null) {
      m_limelight.setLedState(LimelightLedMode.FORCE_ON);
    }
  }

  public void disableLimelight() {
    if (m_limelight != null) {
      m_limelight.setLedState(LimelightLedMode.FORCE_OFF);
    }
  }
}
