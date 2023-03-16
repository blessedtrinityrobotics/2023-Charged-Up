// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.Commands;
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

  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerId);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerId);

  private final AutonomousManager m_autoManager = new AutonomousManager(m_drive, m_elevator, m_arm, m_intake); 

  public RobotContainer() {
    configureBindings();
  }

  private double trimDriveInput(double joystickInput) {
    // Make the lower bound of the stick 
    double inputSign = Math.signum(joystickInput); 
    double interpolatedInput = MathUtil.interpolate(OIConstants.kBaselinePower, OIConstants.kMaxPower, Math.abs(joystickInput));
    return Math.pow(interpolatedInput, 2) * inputSign;
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(
            () -> -trimDriveInput(m_driverController.getLeftY()),
            () -> trimDriveInput(m_driverController.getRightX())));

    m_elevator.setDefaultCommand(m_elevator.liftCommand(() -> -m_operatorController.getLeftY()));
    m_operatorController
    .y()
    .onTrue(
        Commands.run(
            () -> {
              m_arm.setGoal(30);
              m_arm.enable();
            },
            m_arm));

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
    return m_autoManager.getChosenAuto();
  }

}
