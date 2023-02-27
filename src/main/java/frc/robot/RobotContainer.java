// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;

public class RobotContainer {
  private final Drive m_drive = new Drive();
  private final Trajectory autoTrajectory;

  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerId);

  public RobotContainer() {
    configureBindings();

    autoTrajectory = getTrajectory();
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(
        m_drive.tankDriveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getRightY()));
  }

  public Command getAutonomousCommand() {
    return basicPathAuto();
  }

  private Command basicPathAuto() {
    m_drive.resetOdometry(autoTrajectory.getInitialPose());

    return generateRamseteCommand(autoTrajectory)
      .andThen(() -> m_drive.tankDriveVolts(0, 0)); 
  }

  private RamseteCommand generateRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
        trajectory,
        m_drive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts,
        m_drive);
  }

  private Trajectory getTrajectory() {
    String trajectoryJSON = "paths/output/Sample.wpilib.json";
    Trajectory trajectory = new Trajectory();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return trajectory;
  }
}
