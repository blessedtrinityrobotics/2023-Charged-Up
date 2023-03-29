// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShuffleboardConstants;
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
        m_chooser.addOption("Driving Backwards Balance", backwardBalanceAuto());
        m_chooser.addOption("Put High", putHighAuto());
        m_chooser.addOption("Put High & Drive Back", putHighAndDriveAuto());
        m_chooser.addOption("Put High & Back Balance", putHighAndBalanceAuto());
        m_chooser.addOption("High, Drive, & Balance", putHighDriveAndBalanceAuto());

        Shuffleboard.getTab(ShuffleboardConstants.kDriveTab).add("Choose Auto", m_chooser).withSize(2, 1);
    }

    public Command getChosenAuto() {
        return m_chooser.getSelected();
    }

    public Command driveForwardAuto() {
        return new SequentialCommandGroup(
            m_drive.driveDistanceCommand(3, 0.5)
                .withTimeout(3)
        );
    }

    public Command putHighAuto() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                m_arm.runOnce(m_arm::enablePID).andThen(m_arm.highCubeCommand()),
                m_elevator.runOnce(m_elevator::enablePID).andThen(m_elevator.topCommand()),
                m_intake.pullInCommand()),
            m_intake.pushOutCommand(),
            new WaitCommand(0.3),
            m_intake.stopIntakeCommand(),
            m_arm.retractedCommand(),
            new WaitCommand(0.25),
            m_elevator.bottomCommand()
        ); 
    }

    public Command putHighAndDriveAuto() {
        return new SequentialCommandGroup(
            putHighAuto(),
            m_drive.driveBackDistanceCommand(3, 0.6)
                .withTimeout(5)        );
    }

    public Command putHighAndBalanceAuto() {
        return new SequentialCommandGroup(
            putHighAuto(),
            backwardBalanceAuto()
        ); 
    }

    public Command putHighDriveAndBalanceAuto() {
        return new SequentialCommandGroup(
            putHighAuto(),
            m_drive.driveBackDistanceCommand(3.3, 0.55)
                .withTimeout(4),
            new WaitCommand(0.5),
            forwardBalanceAuto()
        );
    }

    public Command backwardBalanceAuto() {
        return new SequentialCommandGroup(
                m_drive.driveUntilBalanced(-0.625, -0.4),
                m_drive.driveDistanceCommand(0.12, 0.4) // it has to drive forward (since it is going backward) to do it properly 
        );
    }

    public Command forwardBalanceAuto() {
        return new SequentialCommandGroup(
                m_drive.driveUntilBalanced(0.625, 0.4),
                m_drive.driveBackDistanceCommand(0.12, 0.4) // it has to drive back to balance properly
        );
    }

    // NOT USED
    public Command dropAndDriveShort() {
        PathPlannerTrajectory path = PathPlanner.loadPath("DriveOutShort", 
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        return new SequentialCommandGroup(
            m_drive.followTrajectoryCommand(path, true)
        );
    }
}
