// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class Intake extends SubsystemBase {
  WPI_TalonSRX m_intakeLeft = new WPI_TalonSRX(ArmConstants.kIntakeLeftId);
  WPI_TalonSRX m_intakeRight = new WPI_TalonSRX(ArmConstants.kIntakeRightId);

  /** Creates a new Intake. */
  public Intake() {
    m_intakeLeft.setNeutralMode(NeutralMode.Brake);
    m_intakeRight.setNeutralMode(NeutralMode.Brake);

    configureIntakeTab();
  }

  public void configureIntakeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab(ShuffleboardConstants.kIntakeTab);
    
  }

  public void intake(double direction) {
    m_intakeLeft.set(ControlMode.PercentOutput, direction);
    m_intakeRight.set(ControlMode.PercentOutput, direction);
  }
  // todo, switch in and out 
  public Command pushOutCommand() {
    return run(() -> intake(IntakeConstants.kDefaultIntakeSpeed)); 
  }

  public Command pullInCommand() {
    return run(() -> intake(-0.4));
  }

  public Command stopIntake() {
    return runOnce(() -> intake(0)); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
