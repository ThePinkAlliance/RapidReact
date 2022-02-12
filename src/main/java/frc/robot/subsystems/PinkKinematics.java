// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import org.ejml.simple.SimpleMatrix;

/** Add your docs here. */
public class PinkKinematics extends SwerveDriveKinematics {

  private final SimpleMatrix m_inverseKinematics;
  private final SimpleMatrix m_forwardKinematics;

  private final int m_numModules;
  private final Translation2d[] m_modules;
  private Translation2d m_prevCoR = new Translation2d();

  private HashMap<Integer, Boolean> inverted = new HashMap<>();

  /**
   * Constructs a swerve drive kinematics object. This takes in a variable number of wheel locations
   * as Translation2ds. The order in which you pass in the wheel locations is the same order that
   * you will receive the module states when performing inverse kinematics. It is also expected that
   * you pass in the module states in the same order when calling the forward kinematics methods.
   *
   * @param wheelsMeters The locations of the wheels relative to the physical center of the robot.
   */
  public PinkKinematics(Translation2d... wheelsMeters) {
    super(wheelsMeters);
    if (wheelsMeters.length < 2) {
      throw new IllegalArgumentException(
        "A swerve drive requires at least two modules"
      );
    }
    m_numModules = wheelsMeters.length;
    m_modules = Arrays.copyOf(wheelsMeters, m_numModules);
    m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

    for (int i = 0; i < m_numModules; i++) {
      m_inverseKinematics.setRow(
        i * 2 + 0,
        0,
        /* Start Data */1,
        0,
        -m_modules[i].getY()
      );
      m_inverseKinematics.setRow(
        i * 2 + 1,
        0,
        /* Start Data */0,
        1,
        +m_modules[i].getX()
      );
    }
    m_forwardKinematics = m_inverseKinematics.pseudoInverse();

    MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
  }

  /**
   * Performs inverse kinematics to return the module states from a desired chassis velocity. This
   * method is often used to convert joystick values into module speeds and angles.
   *
   * <p>This function also supports variable centers of rotation. During normal operations, the
   * center of rotation is usually the same as the physical center of the robot; therefore, the
   * argument is defaulted to that use case. However, if you wish to change the center of rotation
   * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
   *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
   *     component, the robot will rotate around that corner.
   * @return An array containing the module states. Use caution because these module states are not
   *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
   *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(SwerveModuleState[], double)
   *     DesaturateWheelSpeeds} function to rectify this issue.
   */
  @SuppressWarnings("LocalVariableName")
  public SwerveModuleState[] toSwerveModuleStates(
    ChassisSpeeds chassisSpeeds,
    Translation2d centerOfRotationMeters
  ) {
    if (!centerOfRotationMeters.equals(m_prevCoR)) {
      for (int i = 0; i < m_numModules; i++) {
        m_inverseKinematics.setRow(
          i * 2 + 0,
          0,/* Start Data */
          1,
          0,
          -m_modules[i].getY() + centerOfRotationMeters.getY()
        );
        m_inverseKinematics.setRow(
          i * 2 + 1,
          0,/* Start Data */
          0,
          1,
          +m_modules[i].getX() - centerOfRotationMeters.getX()
        );
      }
      m_prevCoR = centerOfRotationMeters;
    }

    var chassisSpeedsVector = new SimpleMatrix(3, 1);
    chassisSpeedsVector.setColumn(
      0,
      0,
      chassisSpeeds.vxMetersPerSecond,
      chassisSpeeds.vyMetersPerSecond,
      chassisSpeeds.omegaRadiansPerSecond
    );

    var moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);
    SwerveModuleState[] moduleStates = new SwerveModuleState[m_numModules];

    for (int i = 0; i < m_numModules; i++) {
      double x = moduleStatesMatrix.get(i * 2, 0);
      double y = moduleStatesMatrix.get(i * 2 + 1, 0);
      double omega = moduleStatesMatrix.get(i * 2 + 2, 0);

      double speed = Math.hypot(x, y);
      Rotation2d angle = new Rotation2d(x, y);

      SmartDashboard.putNumber("s", speed);

      if (speed > 0 && omega > 0) {
        var state = new SwerveModuleState(speed, angle);

        moduleStates[i] = SwerveModuleState.optimize(state, angle);
      } else if (speed < 0 && omega > 0) {
        var state = new SwerveModuleState(-speed, angle);

        moduleStates[i] = SwerveModuleState.optimize(state, angle);
      }
    }

    // for (int i = 0; i < inverted.size(); i++) {
    //   if (inverted.get(i)) {
    //     var oldState = moduleStates[i];
    //     // var oldNeighborState = moduleStates[i + 2];

    //     var newState = new SwerveModuleState(
    //       oldState.speedMetersPerSecond,
    //       oldState.angle
    //     );
    //     // var newNeighborState = new SwerveModuleState(
    //     //   oldNeighborState.speedMetersPerSecond,
    //     //   oldNeighborState.angle
    //     // );

    //     moduleStates[i] = SwerveModuleState.optimize(newState, newState.angle);
    //     // moduleStates[i + 2] =
    //     //   SwerveModuleState.optimize(
    //     //     newNeighborState,
    //     //     newNeighborState.angle
    //     //   );
    //   }
    // }

    return moduleStates;
  }

  /**
   * Performs inverse kinematics. See {@link #toSwerveModuleStates(ChassisSpeeds, Translation2d)}
   * toSwerveModuleStates for more information.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @return An array containing the module states.
   */
  public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
    return toSwerveModuleStates(chassisSpeeds, new Translation2d());
  }

  /**
   * Performs forward kinematics to return the resulting chassis state from the given module states.
   * This method is often used for odometry -- determining the robot's position on the field using
   * data from the real-world speed and angle of each module on the robot.
   *
   * @param wheelStates The state of the modules (as a SwerveModuleState type) as measured from
   *     respective encoders and gyros. The order of the swerve module states should be same as
   *     passed into the constructor of this class.
   * @return The resulting chassis speed.
   */
  public ChassisSpeeds toChassisSpeeds(SwerveModuleState... wheelStates) {
    if (wheelStates.length != m_numModules) {
      throw new IllegalArgumentException(
        "Number of modules is not consistent with number of wheel locations provided in " +
        "constructor"
      );
    }
    var moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

    for (int i = 0; i < m_numModules; i++) {
      var module = wheelStates[i];
      moduleStatesMatrix.set(
        i * 2,
        0,
        module.speedMetersPerSecond * module.angle.getCos()
      );
      moduleStatesMatrix.set(
        i * 2 + 1,
        module.speedMetersPerSecond * module.angle.getSin()
      );

      SmartDashboard.putNumber(
        "sin " + i,
        module.speedMetersPerSecond * module.angle.getSin()
      );
      SmartDashboard.putNumber(
        "cos " + i,
        module.speedMetersPerSecond * module.angle.getCos()
      );
    }

    var chassisSpeedsVector = m_forwardKinematics.mult(moduleStatesMatrix);
    return new ChassisSpeeds(
      chassisSpeedsVector.get(0, 0),
      chassisSpeedsVector.get(1, 0),
      chassisSpeedsVector.get(2, 0)
    );
  }

  /**
   * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
   *
   * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
   * above the max attainable speed for the driving motor on that module. To fix this issue, one can
   * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
   * absolute threshold, while maintaining the ratio of speeds between modules.
   *
   * @param moduleStates Reference to array of module states. The array will be mutated with the
   *     normalized speeds!
   * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module can reach.
   */
  public static void desaturateWheelSpeeds(
    SwerveModuleState[] moduleStates,
    double attainableMaxSpeedMetersPerSecond
  ) {
    double realMaxSpeed = Collections.max(Arrays.asList(moduleStates))
      .speedMetersPerSecond;
    if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
      for (SwerveModuleState moduleState : moduleStates) {
        moduleState.speedMetersPerSecond =
          moduleState.speedMetersPerSecond /
          realMaxSpeed *
          attainableMaxSpeedMetersPerSecond;
      }
    }
  }
}
