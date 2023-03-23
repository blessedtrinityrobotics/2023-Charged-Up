// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class Arm extends SubsystemBase {
  DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort); 

  WPI_TalonFX m_motor = new WPI_TalonFX(ArmConstants.kArmMotorId); 
  ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  PIDController m_controller = new PIDController(ArmConstants.kP,0,0);
  ShuffleboardTab armTab; 
  boolean enabled = false;
  double output = 0.0;  

  /** Creates a new Arm. */
  public Arm() {

    // m_motor.setInverted(true);
    m_motor.setNeutralMode(NeutralMode.Brake);

    m_encoder.setDistancePerRotation(ArmConstants.kEncoderDistancePerRotation);
    m_encoder.setPositionOffset(ArmConstants.kDefaultEncoderOffset);
    // m_encoder.reset();

    disablePID();
    m_controller.setTolerance(0.01);
    m_controller.setSetpoint(getMeasurement());

    configureArmTab();
  }


  private void configureArmTab() {
    armTab = Shuffleboard.getTab(ShuffleboardConstants.kArmTab);
    armTab.add("Offset Command", setEncoderOffsetCommand()).withSize(2, 1);
    armTab.add("Reset Command", runOnce(this::resetEncoder).withName("Reset Encoder")).withSize(2, 1);
    armTab.addDouble("Arm Angle", () -> getMeasurement());
    armTab.addDouble("Abs Angle", () -> m_encoder.getAbsolutePosition());
    armTab.addDouble("Error", m_controller::getPositionError);
    armTab.add("PID", m_controller);
    armTab.addBoolean("At setpoint", m_controller::atSetpoint);
    armTab.addDouble("PID Output", () -> output); 
  }

  // @Override
  // public void useOutput(double output, double setpoint) {
    
  //   // Calculate the feedforward from the sepoint
  //   // double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
  //   double feedforward = 0; 
  //   // Add the feedforward to the PID output to get the motor output
  //   m_motor.setVoltage(output + feedforward);
  // }

  public double getMeasurement() {
    return m_encoder.getAbsolutePosition();  
  }

  public CommandBase setSetpointCommand(DoubleSupplier direction) {
    return runOnce(() -> {
      double newSetpoint = MathUtil.clamp(m_controller.getSetpoint() + direction.getAsDouble() * 0.01, ArmConstants.kLower, ArmConstants.kUpper);
      m_controller.setSetpoint(newSetpoint);
    }); 
  }

  public void enablePID() {
    enabled = true;
  }
  
  public void disablePID() {
    enabled = false; 
  }

  public void resetEncoder() {
    m_encoder.reset();
  }

  public Command moveArmCommand(DoubleSupplier direction) {
    return run(() -> {
      double power = direction.getAsDouble(); 
      if (m_encoder.getDistance() < ArmConstants.kLower) 
        power = Math.max(0, power);
      else if (m_encoder.getDistance() > ArmConstants.kUpper)
        power = Math.min(0, power);

      m_motor.set(ControlMode.PercentOutput, MathUtil.clamp(power, -0.2, 0.2));
    });
  }

  public CommandBase setEncoderOffsetCommand() {
    return runOnce(() -> {
      // resetEncoder();
      // Preferences.setDouble(ArmConstants.kEncoderOffsetKey, m_encoder.getPositionOffset());
    }).withName("Set Encoder Offset");
  }

  @Override
  public void periodic() {
    output = MathUtil.clamp(m_controller.calculate(getMeasurement()), ArmConstants.kMinPower, ArmConstants.kMaxPower); 
    if (enabled == true)
      m_motor.set(ControlMode.PercentOutput, output); 
  }


}
