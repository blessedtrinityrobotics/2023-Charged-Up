// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  MotorControllerGroup m_left = new MotorControllerGroup(
      new WPI_TalonFX(DriveConstants.kFrontLeftDriveId),
      new WPI_TalonFX(DriveConstants.kBackLeftDriveId));

  MotorControllerGroup m_right = new MotorControllerGroup(
      new WPI_TalonFX(DriveConstants.kFrontRightDriveId),
      new WPI_TalonFX(DriveConstants.kBackRightDriveId));

  private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);

  WPI_Pigeon2 m_gyro = new WPI_Pigeon2(DriveConstants.kPigeonId);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  private final DifferentialDriveOdometry m_odometry; 

  /** Creates a new Drivetrain. */
  public Drive() {
    m_right.setInverted(DriveConstants.kRightInverted);
    m_left.setInverted(DriveConstants.kLeftInverted);

    m_leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);

    m_gyro.reset();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);

  }

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
   * @param leftVolts the commanded left output
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
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
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

  public CommandBase tankDriveCommand(DoubleSupplier leftPower, DoubleSupplier rightPower) {
    return run(() -> m_drive.tankDrive(leftPower.getAsDouble(), rightPower.getAsDouble()))
        .withName("tankDrive");
  }

  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> resetEncoders())
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(m_leftEncoder.getDistance(), m_rightEncoder.getDistance())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.tankDrive(0, 0));
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Yaw", m_gyro.getYaw());
      SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getDistance());
      SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getDistance());

      m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
}
