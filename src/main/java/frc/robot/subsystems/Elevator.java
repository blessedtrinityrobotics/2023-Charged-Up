// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class Elevator extends SubsystemBase {
  TimeOfFlight m_rangefinder = new TimeOfFlight(ElevatorConstants.kElevatorRangefinderId);
  WPI_TalonFX m_liftMotor = new WPI_TalonFX(ElevatorConstants.kElevatorMotorId);

  ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
  PIDController pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  double targetPosition = 0;

  /** Creates a new Elevator. */
  public Elevator() {
    m_rangefinder.setRangingMode(RangingMode.Short, 24);
    m_liftMotor.setNeutralMode(NeutralMode.Brake);

    configureElevatorTab();
  }

  private void configureElevatorTab() {
    ShuffleboardTab tab = Shuffleboard.getTab(ShuffleboardConstants.kElevatorTab); 
    tab.addDouble("Elevator Height", m_rangefinder::getRange)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", ElevatorConstants.kLowerRange, "max", ElevatorConstants.kUpperRange));
    
  }

  public CommandBase liftCommand(DoubleSupplier direction) {
    return run(() -> {
      double power = direction.getAsDouble(); 
      //targetPosition = MathUtil.clamp(targetPosition + direction.getAsDouble() * 0.1, ElevatorConstants.kLowerRange, ElevatorConstants.kUpperRange);
      if (m_rangefinder.getRange() < ElevatorConstants.kLowerRange) 
        power = Math.max(0, power);
      else if (m_rangefinder.getRange() > ElevatorConstants.kUpperRange) 
        power = Math.min(0, power);
      
      m_liftMotor.set(ControlMode.PercentOutput, power);
    });
  }

  public void resetTarget() {
    targetPosition = m_rangefinder.getRange() / 1000;
  }

  @Override
  public void periodic() {
    // double output = feedforward.calculate(0.2) + pid.calculate(m_rangefinder.getRange() / 1000, targetPosition);
    // m_liftMotor.setVoltage(output);
    
     SmartDashboard.putNumber("Range", m_rangefinder.getRange());
  }
}
