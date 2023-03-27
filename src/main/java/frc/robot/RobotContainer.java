// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private final Drive m_drive = new Drive();
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();

  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerId);
  private final CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerId);
  
  private final AutonomousManager m_autoManager = new AutonomousManager(m_drive, m_elevator, m_arm, m_intake);

  ShuffleboardTab m_driveTab; 
  DoubleSupplier m_armPIDInputFunc = () -> -InputUtil.applyDeadzone(m_operatorController.getRightY()) * OIConstants.kArmSetpointForcefulness; 

  public RobotContainer() {
    UsbCamera cam = CameraServer.startAutomaticCapture();
    configureDriveTab(cam);
    configureBindings();
  }

  private void configureDriveTab(UsbCamera cam) {
    m_driveTab = Shuffleboard.getTab(ShuffleboardConstants.kDriveTab);
    m_driveTab.add("Cam", cam).withSize(4, 3);
  }

  /**
   * Used to setup controller bindings for the driver and the operator 
   */
  private void configureBindings() {
    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(
            () -> InputUtil.trimDriveInput(m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()),
            () -> -InputUtil.trimDriveInput(m_driverController.getLeftX())));

    m_arm.setDefaultCommand(m_arm.setSetpointCommand(m_armPIDInputFunc));
    m_elevator.setDefaultCommand(m_elevator.setSetpointCommand(() -> -InputUtil.applyDeadzone(m_operatorController.getLeftY())));

    // Driver intake commands 
    m_driverController.leftBumper().onTrue(m_intake.pushOutCommand()).onFalse(m_intake.stopIntakeCommand());
    m_driverController.rightBumper().onTrue(m_intake.pullInCommand()).onFalse(m_intake.stopIntakeCommand());

    // Arm and Elevator PID Control (it starts by default off )
    m_operatorController.y().onTrue(m_arm.runOnce(m_arm::enablePID));
    m_operatorController.x().onTrue(m_arm.runOnce(m_arm::disablePID));
    m_operatorController.a().onTrue(m_elevator.runOnce(m_elevator::enablePID));
    m_operatorController.b().onTrue(m_elevator.runOnce(m_elevator::disablePID)); 

    m_operatorController.leftTrigger().onTrue(m_intake.pushOutCommand()).onFalse(m_intake.stopIntakeCommand());
    m_operatorController.rightTrigger().onTrue(m_intake.pullInCommand()).onFalse(m_intake.stopIntakeCommand());

    // Back turns manual override on, start turns it off 
    m_operatorController.back().onTrue(manualArmControlCommand());  
    m_operatorController.start().onTrue(autoArmControlCommand());  

    // Preset positions for elevator and arm, controlled by the operator 
    m_operatorController.leftBumper().onTrue(m_arm.setRetractedCommand().alongWith(m_elevator.bottomCommand()));
    m_operatorController.rightBumper().onTrue(m_arm.setHorizontalCommand().alongWith(m_elevator.bottomCommand())); 
    m_operatorController.povUp().onTrue(m_arm.setHighCommand().alongWith(m_elevator.topCommand()));
    m_operatorController.povRight().onTrue(m_arm.setHorizontalCommand().alongWith(m_elevator.bottomCommand())); 
    

  }

  public CommandBase manualArmControlCommand() {
    return m_arm.runOnce(() -> {
      m_arm.enableOverride();
      m_arm.setDefaultCommand(m_arm.moveArmCommand(() -> -InputUtil.applyDeadzone(m_operatorController.getRightY())));
    }); 
  }

  public CommandBase autoArmControlCommand() {
    return m_arm.runOnce(() -> {
      m_arm.disableOverride();
      m_arm.setDefaultCommand(m_arm.setSetpointCommand(m_armPIDInputFunc));
    }); 
  }

  public Command getAutonomousCommand() {
    return m_autoManager.getChosenAuto();
  }

}
