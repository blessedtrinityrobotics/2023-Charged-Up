// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;

public class RobotContainer {
  private final Drive m_drivetrain = new Drive();

  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerId);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
        m_drivetrain.tankDriveCommand(
          () -> -m_driverController.getLeftY(), 
          () -> -m_driverController.getRightY()));
  }

  public Command getAutonomousCommand() {
    return m_drivetrain.driveDistanceCommand(0.5, 0.5).withTimeout(10);
  }
}
