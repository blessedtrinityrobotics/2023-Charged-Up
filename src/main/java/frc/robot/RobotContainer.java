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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private final Drive m_drive = new Drive();
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm(); 
  private final Intake m_intake = new Intake();
  private final Trajectory autoTrajectory;
  private final Trajectory backTrajectory; 

  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerId);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerId);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    autoTrajectory = getTrajectory( "paths/output/MeterForward.wpilib.json");
    backTrajectory = getTrajectory( "paths/output/Back.wpilib.json");
    configureBindings();
    configureAutoChooser();
  }

  private void configureAutoChooser() {
    m_chooser.setDefaultOption("Drop & Drive", dropAndDriveBackAuto());
    m_chooser.addOption("Drop & Charge", dropAndBalanceAuto());
    m_chooser.addOption("Drive Forward", basicPathAuto());
    SmartDashboard.putData("Choose Auto", m_chooser);
  }

  private double smoothInput(double val) {
    return Math.pow(val, 3);
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(
            () -> -smoothInput(m_driverController.getLeftY()),
            () -> smoothInput(m_driverController.getRightX())));

    m_elevator.setDefaultCommand(m_elevator.liftCommand(() -> -m_operatorController.getLeftY()));
    m_arm.setDefaultCommand(m_arm.moveArmCommand(() -> m_operatorController.getRightY())); 

    m_driverController.leftBumper().onTrue(m_intake.pushOutCommand()).onFalse(m_intake.stopIntake());
    m_driverController.rightBumper().onTrue(m_intake.pullInCommand()).onFalse(m_intake.stopIntake()); 
    m_operatorController.leftBumper().onTrue(m_intake.pushOutCommand()).onFalse(m_intake.stopIntake());
    m_operatorController.rightBumper().onTrue(m_intake.pullInCommand()).onFalse(m_intake.stopIntake()); 

    m_driverController.a().onTrue(new InstantCommand(m_drive::coastMotors, m_drive));
    m_driverController.b().onTrue(new InstantCommand(m_drive::brakeMotors, m_drive));

    m_operatorController.a().onTrue(new InstantCommand(m_drive::coastMotors, m_drive));
    m_operatorController.b().onTrue(new InstantCommand(m_drive::brakeMotors, m_drive));

    m_operatorController.x().onTrue(new InstantCommand(m_arm::resetEncoder, m_arm)); 
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  private Command basicPathAuto() {
    // should we reset pose?
    m_drive.resetOdometry(autoTrajectory.getInitialPose());
    
    return generateRamseteCommand(autoTrajectory)
      .withTimeout(3)
      .andThen(() -> m_drive.tankDriveVolts(0, 0)); 
  }

  private Command dropAndBalanceAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(m_drive::brakeMotors, m_drive),
      m_drive.driveBackDistanceCommand(0.2, 0.5),
      m_drive.driveDistanceCommand(0.1, 0.5),
      m_intake.pushOutCommand().withTimeout(1),
      m_intake.stopIntake().withTimeout(1),
      // todo: add drive up and then drive until balanced, just do it with commands 
      //generateRamseteCommand(backTrajectory).withTimeout(3),// change to other
      m_drive.driveBackDistanceCommand(3, 0.4),
      new InstantCommand(() -> m_drive.tankDriveVolts(0, 0))
    );
  }

  private Command dropAndDriveBackAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(m_drive::brakeMotors, m_drive),
      m_drive.driveBackDistanceCommand(0.2, 0.5),
      m_drive.driveDistanceCommand(0.1, 0.5),
      m_intake.pushOutCommand().withTimeout(1),
      m_intake.stopIntake().withTimeout(1),
      //generateRamseteCommand(backTrajectory).withTimeout(3),// change to other
      m_drive.driveBackDistanceCommand(3, 0.4),
      new InstantCommand(() -> m_drive.tankDriveVolts(0, 0))
    );
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

  private Trajectory getTrajectory(String traj) {
    String trajectoryJSON = traj;
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
