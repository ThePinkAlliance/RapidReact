// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoMidClimb;
import frc.robot.commands.CollectGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.JoystickClimb;
import frc.robot.commands.LeaveTarmack;
import frc.robot.commands.MoveTower;
import frc.robot.commands.ShootLeaveTarmac;
import frc.robot.commands.ShootLeaveTarmacCollectShoot;
import frc.robot.commands.SpinUpShooter;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Shooter;

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
  private final Joystick gamepad_tower = new Joystick(1);
  private final Base m_base = new Base();
  //private final Limelight m_limelight = new Limelight();
  private final Collector m_collector = new Collector();
  private final Shooter m_shooter = new Shooter();
  private final Climbers m_climbers = new Climbers();
  //DASHBOARD MUST BE LAST SUBSYSTEM INSTANTIATED
  //private final Dashboard m_dashboard = new Dashboard(m_base, m_collector, m_shooter, null);

  Trajectory trajectory = new Trajectory();
  //ShuffleboardTab driverDashboard = Shuffleboard.getTab("Dashboard");
  SendableChooser<SelectableTrajectory> selectedPath = new SendableChooser<SelectableTrajectory>();

  private final SelectableTrajectory LeaveTarmac = new SelectableTrajectory(
    "Leave Tarmac",
    new LeaveTarmack(m_base)
  );

  private final SelectableTrajectory ShootLeaveTarmac = new SelectableTrajectory(
    "Shoot Leave Tarmac",
    new ShootLeaveTarmac(m_base, m_shooter, m_collector)
  );

  private final SelectableTrajectory ShootLeaveTarmacCollectShoot = new SelectableTrajectory(
    "Shoot Leave Tarmac Collect And Shoot",
    new ShootLeaveTarmacCollectShoot(m_base, m_shooter, m_collector)
  );

  private final SelectableTrajectory autoMidClimb = new SelectableTrajectory(
    "AutoMidClimb",
    new AutoMidClimb(m_base, m_climbers)
  );

  /**
   * This contains all the trajectories that can be selected from the dashboard.
   */
  private final SelectableTrajectory[] trajectories = {
    LeaveTarmac,
    ShootLeaveTarmac,
    ShootLeaveTarmacCollectShoot,
    autoMidClimb,
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    for (SelectableTrajectory t : trajectories) {
      if (t.location == ShootLeaveTarmac.name) {
        selectedPath.setDefaultOption(t.name, t);
      } else {
        selectedPath.addOption(t.name, t);
      }
    }

    //driverDashboard.add(selectedPath);
    SmartDashboard.putData(selectedPath);

    // for now select leave blue 1 for testing

    //Initialize and publish for the first time.  Default command of dashboard handles thereafter.
    //m_dashboard.initialize();
    //m_dashboard.publishDashboard();
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_VELOCITY,
      this.m_shooter.getMotorOutputPercent()
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_TARGET_RPMS,
      Shooter.SHOOTER_POWER_CLOSE_HIGH
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_RPMS,
      m_shooter.getMotorRpms()
    );
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, false);
    SmartDashboard.putNumber(
      Dashboard.DASH_CLIMBER_LONG_ARM_POSITION,
      m_climbers.longClimberModule.getPosition()
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_CLIMBER_SHORT_ARM_POSITION,
      m_climbers.shortClimberModule.getPosition()
    );
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
    //base controller
    //left joystick

    this.m_base.setDefaultCommand(new Drive(m_base, this.gamepad_base));
    this.m_climbers.setDefaultCommand(
        new JoystickClimb(m_climbers, this.gamepad_tower)
      );
    //this.m_dashboard.setDefaultCommand(new DashboardPublish(m_dashboard));

    //Shooter - Shoot - move tower to push ball up to shooter
    new JoystickButton(gamepad_tower, Constants.JOYSTICK_BUTTON_X)
    .whenPressed(
        new MoveTower(
          m_collector,
          Shooter.SHOOTER_POWER_CLOSE_HIGH_V2,
          Constants.JOYSTICK_BUTTON_X,
          gamepad_tower,
          true
        )
      );
    new JoystickButton(gamepad_tower, Constants.JOYSTICK_BUTTON_A)
    .whenPressed(
        new SpinUpShooter(
          m_shooter,
          Shooter.SHOOTER_POWER_CLOSE_HIGH_V2,
          Constants.JOYSTICK_BUTTON_A,
          gamepad_tower
        )
      );
    //Collector Intake
    new JoystickButton(gamepad_base, Constants.JOYSTICK_RIGHT_BUMPER)
    .whenPressed(
        new CollectGroup(
          m_collector,
          gamepad_base,
          Constants.JOYSTICK_RIGHT_BUMPER,
          true
        )
      );
    //Collector Outtake
    new JoystickButton(gamepad_base, Constants.JOYSTICK_LEFT_BUMPER)
    .whenPressed(
        new CollectGroup(
          m_collector,
          gamepad_base,
          Constants.JOYSTICK_LEFT_BUMPER,
          false
        )
      );
    //Climbers
    // new JoystickButton(gamepad_tower, Constants.JOYSTICK_BUTTON_Y)
    // .whenPressed(
    //     new MoveShortArms(
    //       m_climbers,
    //       ClimberModule.SHORT_ARM_MID_CLIMB_START,
    //       MoveShortArms.ARM_MOVE_UP
    //     )
    //     // .andThen(
    //     //     new MoveLongArms(
    //     //       m_climbers,
    //     //       ClimberModule.LONG_ARM_MID_CLIMB_START,s
    //     //       MoveLongArms.ARM_MOVE_UP
    //     //     )
    //     //   )
    //     //new ClimbDrive(m_base, m_climbers, 0, 0.7, false)
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
    if (selectedPath.getSelected() == null) {
      selectedPath.setDefaultOption(ShootLeaveTarmac.name, ShootLeaveTarmac);
    }

    return selectedPath.getSelected().getDefualtCommand();
  }

  // public void enableLimelight() {
  //   if (m_limelight != null) {
  //     m_limelight.setLedState(LimelightLedMode.FORCE_ON);
  //   }
  // }

  // public void disableLimelight() {
  //   if (m_limelight != null) {
  //     m_limelight.setLedState(LimelightLedMode.FORCE_OFF);
  //   }
  // }

  public void testInit() {
    m_base.setPodAngles(0);
  }
}
