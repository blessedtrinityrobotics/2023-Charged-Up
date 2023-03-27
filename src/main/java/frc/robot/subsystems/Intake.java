// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class Intake extends SubsystemBase {
  WPI_TalonSRX m_intakeLeft = new WPI_TalonSRX(ArmConstants.kIntakeLeftId);
  WPI_TalonSRX m_intakeRight = new WPI_TalonSRX(ArmConstants.kIntakeRightId);
  boolean intaking = false; 
  boolean outaking = false; 

  /** Creates a new Intake. */
  public Intake() {
    m_intakeLeft.setNeutralMode(NeutralMode.Brake);
    m_intakeRight.setNeutralMode(NeutralMode.Brake);

    configureIntakeTab();
  }

  public void configureIntakeTab() {
    ShuffleboardTab intakeTab = Shuffleboard.getTab(ShuffleboardConstants.kIntakeTab);
    ShuffleboardTab driveTab = Shuffleboard.getTab(ShuffleboardConstants.kDriveTab);
    driveTab.addBoolean("Intaking", () -> intaking);
    driveTab.addBoolean("Outaking", () -> outaking);
    
    intakeTab.addDouble("Motor Output", m_intakeLeft::get); 
  }

  public void intake(double direction) {
    m_intakeLeft.set(ControlMode.PercentOutput, direction);
    m_intakeRight.set(ControlMode.PercentOutput, direction);
  }

  public CommandBase intakeCommand(DoubleSupplier direction) {
    return runOnce(() -> intake(direction.getAsDouble())); 
  }

  public Command pushOutCommand() {
    intaking = false; 
    outaking = true; 
    return runOnce(() -> intake(IntakeConstants.kDefaultOutakeSpeed)); 
  }

  public Command pullInCommand() {
    intaking = true; 
    outaking = false; 
    return runOnce(() -> intake(IntakeConstants.kDefaultIntakeSpeed));
  }

  public Command stopIntakeCommand() {
    intaking = false;
    outaking = false; 
    return runOnce(() -> intake(0)); 

  }

}
