// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class Drive extends SubsystemBase {
  WPI_TalonFX m_frontLeft = new WPI_TalonFX(DriveConstants.kFrontLeftDriveId);
  WPI_TalonFX m_backLeft = new WPI_TalonFX(DriveConstants.kBackLeftDriveId);
  WPI_TalonFX m_frontRight = new WPI_TalonFX(DriveConstants.kFrontRightDriveId);
  WPI_TalonFX m_backRight = new WPI_TalonFX(DriveConstants.kBackRightDriveId);

  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_backLeft);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_backRight);

  private final Encoder m_leftEncoder = new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);

  WPI_Pigeon2 m_gyro = new WPI_Pigeon2(DriveConstants.kPigeonId);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  DifferentialDriveOdometry m_odometry;
  boolean m_braked = false; 

  /** Creates a new Drivetrain. */
  public Drive() {
    m_right.setInverted(DriveConstants.kRightInverted);
    m_left.setInverted(DriveConstants.kLeftInverted);

    m_leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);

    m_gyro.reset();
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);
    brakeMotors();

    configureDriveTab();
  }

  private void configureDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab(ShuffleboardConstants.kDriveTab);
    ShuffleboardTab autoTab = Shuffleboard.getTab(ShuffleboardConstants.kAutoTab); 
    ShuffleboardLayout encoderLayout = autoTab.getLayout("Encoders", BuiltInLayouts.kList)
      .withSize(2, 2);
    encoderLayout.add("Left Encoder", m_leftEncoder);
    encoderLayout.add("Right Encoder", m_rightEncoder);
    // driveTab.add("Gyro", m_gyro);
    autoTab.addDouble("Roll", () -> getRoll());
    driveTab.addBoolean("Motors Braked", () -> m_braked);
    // driveTab.addCamera("Webcam", "USB Camera 0"); 
  }

  public CommandBase arcadeDriveCommand(DoubleSupplier leftPower, DoubleSupplier rightPower) {
    return run(() -> m_drive.arcadeDrive(leftPower.getAsDouble(), rightPower.getAsDouble()))
        .withName("tankDrive");
  }

  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
        () -> resetEncoders())
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () -> Math.max(m_leftEncoder.getDistance(), m_rightEncoder.getDistance()) >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.tankDrive(0, 0));
  }

  public CommandBase driveBackDistanceCommand(double distanceMeters, double speed) {
    double absSpeed = Math.abs(speed); 
    return runOnce(
        () -> resetEncoders())
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(-absSpeed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () -> Math.min(m_leftEncoder.getDistance(), m_rightEncoder.getDistance()) <= -distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.tankDrive(0, 0));
  }

  public CommandBase driveUntilBalanced(double intialPower, double finalPower) {
    return runOnce(
        () -> resetGyro())
        // Drive forward at specified speed
        .andThen(
            run(() -> m_drive.arcadeDrive(intialPower, 0))
                .until(() -> Math.abs(getRoll()) > AutoConstants.kRollBackDegrees))
        .andThen(
            run(() -> m_drive.arcadeDrive(finalPower, 0))
                .until(() -> Math.abs(getRoll()) < AutoConstants.kBalancedDegrees))
        .finallyDo(interrupted -> m_drive.arcadeDrive(0, 0));
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void brakeMotors() {
    m_frontLeft.setNeutralMode(NeutralMode.Brake);
    m_backLeft.setNeutralMode(NeutralMode.Brake);
    m_frontRight.setNeutralMode(NeutralMode.Brake);
    m_backRight.setNeutralMode(NeutralMode.Brake);
    
    m_braked = true;
  }

  public void coastMotors() {
    m_frontLeft.setNeutralMode(NeutralMode.Coast);
    m_backLeft.setNeutralMode(NeutralMode.Coast);
    m_frontRight.setNeutralMode(NeutralMode.Coast);
    m_backRight.setNeutralMode(NeutralMode.Coast);
    m_braked = false;
  }

  public double getRoll() {
    return m_gyro.getRoll() - DriveConstants.kFlatGyroRoll;
  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

// ===================================================================================================
// PROBABLY DON'T NEED TO TOUCH METHODS BELOW THIS LINE... MOST ARE PREGENERATED
// ===================================================================================================

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left.setVoltage(leftVolts);
    m_right.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(DriveConstants.kPDriveVel, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(DriveConstants.kPDriveVel, 0, 0), // Right controller (usually the same values as left controller)
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
        
    );
  }
}
