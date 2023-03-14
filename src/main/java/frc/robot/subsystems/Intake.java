// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Intake extends SubsystemBase {
  WPI_TalonSRX m_intakeLeft = new WPI_TalonSRX(ArmConstants.kIntakeLeftId);
  WPI_TalonSRX m_intakeRight = new WPI_TalonSRX(ArmConstants.kIntakeRightId);

  public double intakeSpeed = 0.5;
  /** Creates a new Intake. */
  public Intake() {}

  public void intake(double direction) {
    m_intakeLeft.set(ControlMode.PercentOutput, direction);
    m_intakeRight.set(ControlMode.PercentOutput, direction);

    if (direction < 0) 
      SmartDashboard.putBoolean("Intaking", true);
    else if (direction > 0)
      SmartDashboard.putBoolean("Outaking", true);
    else
      SmartDashboard.putBoolean("Outaking", false);
      SmartDashboard.putBoolean("Intaking", false);
  }
  // todo, switch in and out 
  public Command pushOutCommand() {
    return run(() -> intake(intakeSpeed)); 
  }

  public Command pullInCommand() {
    return run(() -> intake(-intakeSpeed));
  }

  public Command stopIntake() {
    return runOnce(() -> intake(0)); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
