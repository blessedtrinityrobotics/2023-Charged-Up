// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  DoubleSupplier m_armPIDInputFunc = () -> -Utils.applyDeadzone(m_operatorController.getRightY()) * OIConstants.kArmSensitivity; 

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
            () -> m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis(),
            () -> -Utils.trimDriveInput(m_driverController.getLeftX())));

    m_arm.setDefaultCommand(m_arm.setSetpointCommand(m_armPIDInputFunc));
    m_elevator.setDefaultCommand(m_elevator.setGoalCommand(() -> -Utils.applyDeadzone(m_operatorController.getLeftY()) * OIConstants.kElevatorSensitivity));

    // Driver and Operator intake and outtake  
    m_driverController.x().onTrue(m_intake.pushOutCommand()).onFalse(m_intake.stopIntakeCommand());
    m_driverController.a().onTrue(m_intake.pullInCommand()).onFalse(m_intake.stopIntakeCommand());
    // Note how operator uses the triggers while driver uses x and a 
    m_operatorController.leftTrigger().onTrue(m_intake.pushOutCommand()).onFalse(m_intake.stopIntakeCommand());
    m_operatorController.rightTrigger().onTrue(m_intake.pullInCommand()).onFalse(m_intake.stopIntakeCommand());
    m_operatorController.y().onTrue(m_intake.holdInCommand()).onFalse(m_intake.stopIntakeCommand()); 
    m_operatorController.b().onTrue(m_intake.slowOutCommand()).onFalse(m_intake.stopIntakeCommand()); 

    // Brake toggles  
    m_driverController.b().onTrue(m_drive.runOnce(m_drive::brakeMotors));
    m_driverController.y().onTrue(m_drive.runOnce(m_drive::coastMotors)); 

    // Arm/Elevator PID Control (it starts by default off 
    m_operatorController.a().onTrue(m_arm.runOnce(m_arm::enablePID).alongWith(m_elevator.runOnce(m_elevator::enablePID))); 
    m_operatorController.x().onTrue(m_arm.runOnce(m_arm::disablePID).alongWith(m_elevator.runOnce(m_elevator::disablePID))); 

    // Operator Manual Override toggle  
    m_operatorController.back().onTrue(manualArmControlCommand()); // Manual ON 
    m_operatorController.start().onTrue(autoArmControlCommand());  // Manual OFF

    // Preset positions for elevator and arm, controlled by the operator 
    m_operatorController.leftBumper().onTrue(m_arm.retractedCommand().alongWith(m_elevator.bottomCommand())); // Retracted away
    m_operatorController.rightBumper().onTrue(m_arm.horizontalCommand().alongWith(m_elevator.bottomCommand())); // Ready for pickup 
    m_operatorController.povUp().onTrue(m_arm.highCubeCommand().alongWith(m_elevator.topCommand())); // High Cube
    m_operatorController.povRight().onTrue(m_arm.highCubeCommand().alongWith(m_elevator.midCubeCommand())); // Mid Cube
    m_operatorController.povLeft().onTrue(m_arm.midConeCommand().alongWith(m_elevator.midConeCommand())); 
    m_operatorController.povDown().onTrue(m_arm.humanStationCommand().alongWith(m_elevator.topCommand()));
  }

  /**
   * Make arm manually controlled
   */
  public CommandBase manualArmControlCommand() {
    return m_arm.runOnce(() -> {
      m_arm.enableOverride();
      m_arm.setDefaultCommand(m_arm.moveArmCommand(() -> -Utils.applyDeadzone(m_operatorController.getRightY())));
    }); 
  }

  /*
   * Go back to manual PID control 
   */
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
