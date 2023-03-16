// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class Drive extends SubsystemBase {
  WPI_TalonFX fl = new WPI_TalonFX(DriveConstants.kFrontLeftDriveId);
  WPI_TalonFX bl = new WPI_TalonFX(DriveConstants.kBackLeftDriveId);
  WPI_TalonFX fr = new WPI_TalonFX(DriveConstants.kFrontRightDriveId);
  WPI_TalonFX br = new WPI_TalonFX(DriveConstants.kBackRightDriveId);

  MotorControllerGroup m_left = new MotorControllerGroup(fl, bl);
  MotorControllerGroup m_right = new MotorControllerGroup(fr, br);

  private double reverseMultipler = 1;

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

  /** Creates a new Drivetrain. */
  public Drive() {
    m_right.setInverted(DriveConstants.kRightInverted);
    m_left.setInverted(DriveConstants.kLeftInverted);

    m_leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);

    m_gyro.reset();
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);

    configureDriveTab();
  }

  private void configureDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab(ShuffleboardConstants.kDriveTab);
    ShuffleboardLayout encoderLayout = driveTab.getLayout("Encoders", BuiltInLayouts.kList);
    encoderLayout.add("Left Encoder", m_leftEncoder);
    encoderLayout.add("Right Encoder", m_rightEncoder);
    driveTab.add("Gyro", m_gyro);
    driveTab.addDouble("Roll", () -> getRoll());

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
    return runOnce(
        () -> resetEncoders())
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(-speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () -> Math.min(m_leftEncoder.getDistance(), m_rightEncoder.getDistance()) <= -distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.tankDrive(0, 0));
  }

  public CommandBase driveUntilBalanced(double power) {
    return runOnce(
        () -> resetGyro())
        // Drive forward at specified speed
        .andThen(
            run(() -> m_drive.arcadeDrive(power, 0))
                .until(() -> getRoll() > AutoConstants.kRollBackDegrees))
        .andThen(
            run(() -> m_drive.arcadeDrive(power, 0))
                .until(() -> getRoll() < AutoConstants.kBalancedDegrees))
        .andThen(driveBackDistanceCommand(0.2, power))
        .finallyDo(interrupted -> m_drive.arcadeDrive(0, 0));
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void brakeMotors() {
    fl.setNeutralMode(NeutralMode.Brake);
    bl.setNeutralMode(NeutralMode.Brake);
    fr.setNeutralMode(NeutralMode.Brake);
    br.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putBoolean("Motors Braked", true);
  }

  public void reverseDirection() {
    reverseMultipler = -1;
  }

  public void forwardDirection() {
    reverseMultipler = 1;
  }

  public void coastMotors() {
    fl.setNeutralMode(NeutralMode.Coast);
    bl.setNeutralMode(NeutralMode.Coast);
    fr.setNeutralMode(NeutralMode.Coast);
    br.setNeutralMode(NeutralMode.Coast);
    
    SmartDashboard.putBoolean("Motors Braked", false);

  }

  public double getRoll() {
    return m_gyro.getRoll() - DriveConstants.kFlatGyroRoll;
  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
}
