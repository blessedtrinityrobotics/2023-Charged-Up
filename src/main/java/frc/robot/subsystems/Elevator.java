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

  PIDController m_controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  boolean enabled = false; 
  double output = 0; 

  /** Creates a new Elevator. */
  public Elevator() {
    m_rangefinder.setRangingMode(RangingMode.Short, 24);
    m_liftMotor.setNeutralMode(NeutralMode.Brake);
    m_controller.setTolerance(10.0);
    
    disablePID();
    configureElevatorTab();
    m_controller.setSetpoint(ElevatorConstants.kLower);
  }

  private void configureElevatorTab() {
    ShuffleboardTab tab = Shuffleboard.getTab(ShuffleboardConstants.kElevatorTab); 
    tab.addDouble("Elevator Height", m_rangefinder::getRange)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", ElevatorConstants.kLower, "max", ElevatorConstants.kUpper));
    tab.add("PID", m_controller);
    tab.addBoolean("At setpoint", m_controller::atSetpoint); 
    
  }

  /**
   * Moves elevator up all the way   
   */
  public Command topCommand() {
    return run(() -> m_controller.setSetpoint(ElevatorConstants.kUpper))
      .until(m_controller::atSetpoint); 
  }

  

  /**
   * Moves elevator down all the way  
   */
  public Command bottomCommand() {
    return run(() -> m_controller.setSetpoint(ElevatorConstants.kLower))
      .until(m_controller::atSetpoint);
  }

  public CommandBase liftCommand(DoubleSupplier direction) {
    return run(() -> {
      double power = direction.getAsDouble(); 
      if (m_rangefinder.getRange() < ElevatorConstants.kLower) 
        power = Math.max(0, power);
      else if (m_rangefinder.getRange() > ElevatorConstants.kUpper) 
        power = Math.min(0, power);
      
      m_liftMotor.set(ControlMode.PercentOutput, power);
    });
  }

  public CommandBase setSetpointCommand(DoubleSupplier direction) {
    return run(() -> {
      double newSetpoint = MathUtil.clamp(m_controller.getSetpoint() + direction.getAsDouble() * 5, ElevatorConstants.kLower, ElevatorConstants.kUpper);
      m_controller.setSetpoint(newSetpoint);
    }); 
  }

  public void enablePID() {
    enabled = true;
  }
  
  public void disablePID() {
    enabled = false; 
  }


  @Override
  public void periodic() {
    if (enabled)
      output = ElevatorConstants.kElevatorFeedforward 
        + MathUtil.clamp(m_controller.calculate(getMeasurement()), ElevatorConstants.kMinPower, ElevatorConstants.kMaxPower); 
      m_liftMotor.set(ControlMode.PercentOutput, output); 
  }

  private double getMeasurement() {
    return m_rangefinder.getRange();
  }
}
