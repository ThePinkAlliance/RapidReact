// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Base extends SubsystemBase {
        private AHRS gyro = new AHRS();

        private ShuffleboardTab tab;

        private final SwerveModule frontLeftModule;
        private final SwerveModule frontRightModule;
        private final SwerveModule backLeftModule;
        private final SwerveModule backRightModule;

        public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        new Translation2d(Constants.Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.Base.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(Constants.Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -Constants.Base.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(-Constants.Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.Base.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(-Constants.Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -Constants.Base.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
                        Rotation2d.fromDegrees(0));

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        private SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        private Mk4ModuleConfiguration configuration = new Mk4ModuleConfiguration();

        /** Creates a new Base. */
        public Base() {

                this.tab = Shuffleboard.getTab("debug");

                this.backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2,
                                                4).withPosition(0,
                                                                0),
                                configuration,
                                Constants.Base.motorRatio,
                                Constants.Base.BACK_RIGHT_DRIVE_MOTOR_PORT,
                                Constants.Base.BACK_RIGHT_STEER_MOTOR_PORT,
                                Constants.Base.BACK_RIGHT_CANCODER_ID,
                                Constants.Base.BACK_RIGHT_MODULE_STEER_OFFSET);

                this.backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2,
                                                4).withPosition(1,
                                                                0),
                                configuration,
                                Constants.Base.motorRatio,
                                Constants.Base.BACK_LEFT_DRIVE_MOTOR_PORT,
                                Constants.Base.BACK_LEFT_STEER_MOTOR_PORT,
                                Constants.Base.BACK_LEFT_CANCODER_ID,
                                Constants.Base.BACK_LEFT_MODULE_STEER_OFFSET);

                this.frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2,
                                                4).withPosition(2,
                                                                0),
                                configuration,
                                Constants.Base.motorRatio,
                                Constants.Base.FRONT_LEFT_DRIVE_MOTOR_PORT,
                                Constants.Base.FRONT_LEFT_STEER_MOTOR_PORT,
                                Constants.Base.FRONT_LEFT_CANCODER_ID,
                                Constants.Base.FRONT_LEFT_MODULE_STEER_OFFSET);

                this.frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2,
                                                4).withPosition(3,
                                                                0),
                                configuration,
                                Constants.Base.motorRatio,
                                Constants.Base.BACK_RIGHT_DRIVE_MOTOR_PORT,
                                Constants.Base.BACK_RIGHT_STEER_MOTOR_PORT,
                                Constants.Base.BACK_RIGHT_CANCODER_ID,
                                Constants.Base.BACK_RIGHT_MODULE_STEER_OFFSET);
        }

        public void drive(ChassisSpeeds speeds) {
                this.chassisSpeeds = speeds;
        }

        public void setStates(SwerveModuleState[] states) {
                // This method will be called once per scheduler run
                this.frontLeftModule.set(
                                (states[0].speedMetersPerSecond / Constants.Base.MAX_VELOCITY_METERS_PER_SECOND) *
                                                -1.0,
                                states[0].angle.getDegrees());
                this.frontRightModule.set((states[1].speedMetersPerSecond
                                / Constants.Base.MAX_VELOCITY_METERS_PER_SECOND) * 1.0,
                                states[1].angle.getDegrees());
                this.backLeftModule.set((states[2].speedMetersPerSecond
                                / Constants.Base.MAX_VELOCITY_METERS_PER_SECOND) * 1.0,
                                states[2].angle.getDegrees());
                this.backRightModule.set((states[3].speedMetersPerSecond
                                / Constants.Base.MAX_VELOCITY_METERS_PER_SECOND) * -1.0,
                                states[3].angle.getDegrees());

                odometry.update(getRotation(), this.states);
        }

        public ChassisSpeeds getChassisSpeeds() {
                return chassisSpeeds;
        }

        public ChassisSpeeds getChassisSpeeds(double x, double y) {
                return chassisSpeeds;
        }

        public void zeroGyro() {
                gyro.reset();
        }

        public void resetOdometry(Pose2d pose) {
                this.odometry.resetPosition(pose, this.getRotation());
        }

        public Rotation2d getRotation() {
                return Rotation2d.fromDegrees(gyro.getFusedHeading());
                // return Rotation2d.fromDegrees(0);
        }

        public boolean isInverted() {
                return getRotation().getDegrees() <= 190 && getRotation().getDegrees() > 90
                                || getRotation().getDegrees() >= 290 && getRotation().getDegrees() < 90;
        }

        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        public double getDirection() {
                if (isInverted()) {
                        return 1.0;
                }

                return -1.0;
        }

        @Override
        public void periodic() {
                // This method will be called once per scheduler run
                this.frontLeftModule.set(
                                (states[0].speedMetersPerSecond / Constants.Base.MAX_VELOCITY_METERS_PER_SECOND) *
                                                -1.0,
                                states[0].angle.getDegrees());
                this.frontRightModule.set((states[1].speedMetersPerSecond
                                / Constants.Base.MAX_VELOCITY_METERS_PER_SECOND) * 1.0,
                                states[1].angle.getDegrees());
                this.backLeftModule.set((states[2].speedMetersPerSecond
                                / Constants.Base.MAX_VELOCITY_METERS_PER_SECOND) * 1.0,
                                states[2].angle.getDegrees());
                this.backRightModule.set((states[3].speedMetersPerSecond
                                / Constants.Base.MAX_VELOCITY_METERS_PER_SECOND) * -1.0,
                                states[3].angle.getDegrees());

                odometry.update(getRotation(), this.states);
        }
}
