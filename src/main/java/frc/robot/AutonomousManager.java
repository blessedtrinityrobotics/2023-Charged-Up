// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
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

/**
 * Organizational abstraction for the automous phase to de-clutter the RobotContainer
 * 
 */
public class AutonomousManager {
    private final Drive m_drive;
    private final Elevator m_elevator;
    private final Arm m_arm;
    private final Intake m_intake;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public AutonomousManager(Drive drive, Elevator elevator, Arm arm, Intake intake) {
        m_drive = drive;
        m_elevator = elevator;
        m_arm = arm;
        m_intake = intake;

        configureAutoChooser();
    }

    private void configureAutoChooser() {
        m_chooser.setDefaultOption("Drive", driveForwardAuto());
        m_chooser.addOption("Balance", driveAndBalanceAuto());
        m_chooser.addOption("Drop Low & Balance", dropAndBalanceAuto());
    }

    public Command getChosenAuto() {
        return m_chooser.getSelected();
    }
    

    public Command dropAndDriveShort() {
        PathPlannerTrajectory path = PathPlanner.loadPath("DriveOutShort", 
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        return new SequentialCommandGroup(
            m_drive.followTrajectoryCommand(path, true)
        );
    }

    public Command driveForwardAuto() {
        return new SequentialCommandGroup(
            m_drive.driveDistanceCommand(3, 0.5).withTimeout(3),
            m_drive.stopMotorCommand()
        );
    }

    public Command dropAndBalanceAuto() {
        return new SequentialCommandGroup(
                new InstantCommand(m_drive::brakeMotors, m_drive),
                m_drive.driveBackDistanceCommand(0.2, 0.5),
                m_drive.driveDistanceCommand(0.1, 0.5),
                m_intake.pushOutCommand().withTimeout(1),
                m_intake.stopIntake().withTimeout(1),
				m_drive.driveUntilBalanced(-0.4));
    }

    public Command driveAndBalanceAuto() {
        return new SequentialCommandGroup(
                new InstantCommand(m_drive::brakeMotors, m_drive),
                m_drive.driveUntilBalanced(0.4));
    }

    public Command dropAndDriveBackAuto() {
        return new SequentialCommandGroup(
                new InstantCommand(m_drive::brakeMotors, m_drive),
                m_drive.driveBackDistanceCommand(0.2, 0.5),
                m_drive.driveDistanceCommand(0.1, 0.5),
                m_intake.pushOutCommand().withTimeout(1),
                m_intake.stopIntake().withTimeout(1),
                // generateRamseteCommand(backTrajectory).withTimeout(3),// change to other
                m_drive.driveBackDistanceCommand(3, 0.4),
                m_drive.stopMotorCommand());
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
