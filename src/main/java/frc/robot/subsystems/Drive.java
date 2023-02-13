// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

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

  WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  /** Creates a new Drivetrain. */
  public Drive() {
    m_right.setInverted(DriveConstants.kRightInverted);
    m_left.setInverted(DriveConstants.kLeftInverted);

    m_leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);

    m_gyro.reset();
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public CommandBase tankDriveCommand(DoubleSupplier leftPower, DoubleSupplier rightPower) {
    return run(() -> m_drive.tankDrive(leftPower.getAsDouble(), rightPower.getAsDouble()))
        .withName("tankDrive");
  }

  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              // Reset encoders at the start of the command
              m_leftEncoder.reset();
              m_rightEncoder.reset();
            })
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(m_leftEncoder.getDistance(), m_rightEncoder.getDistance())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Yaw", m_gyro.getYaw());
      SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getDistance());
      SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getDistance());
  }
}
