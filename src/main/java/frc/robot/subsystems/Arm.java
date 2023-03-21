// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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

  WPI_TalonSRX m_motor = new WPI_TalonSRX(ArmConstants.kArmMotorId); 
  ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  PIDController m_controller = new PIDController(ArmConstants.kP,0,0);
  boolean enabled = false; 
  ShuffleboardTab armTab; 

  /** Creates a new Arm. */
  public Arm() {
    Preferences.initDouble(ArmConstants.kEncoderOffsetKey, ArmConstants.kDefaultEncoderOffset);
    m_motor.setInverted(true);
    m_encoder.setDistancePerRotation(ArmConstants.kEncoderDistancePerRotation);
    m_encoder.reset();
    m_encoder.setPositionOffset(Preferences.getDouble(ArmConstants.kEncoderOffsetKey, ArmConstants.kDefaultEncoderOffset));
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_controller.setTolerance(0.01);
    configureArmTab();

    // Start arm at rest in neutral position
    // setGoal(0);
  }

  private void configureArmTab() {
    armTab = Shuffleboard.getTab(ShuffleboardConstants.kArmTab);
    armTab.add("Offset Command", setEncoderOffsetCommand()).withSize(2, 1);
    armTab.add("Reset Command", runOnce(this::resetEncoder)).withSize(2, 1);
    armTab.addDouble("Arm Angle", () -> getMeasurement());
    armTab.addDouble("Error", m_controller::getPositionError);
    // armTab.addDouble("Error", getController()::getPositionError);
    armTab.addBoolean("At setpoint", m_controller::atSetpoint);
    // armTab.addDouble("kP", getController()::getP);

      // .withWidget(BuiltInWidgets.kDial)
      // .withProperties(Map.of("min", ArmConstants.kLower, "max", ArmConstants.kUpper)); 
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
    return m_encoder.getDistance();  
  }

  public void setSetpoint(double s) {
    m_controller.setSetpoint(s);
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

  public CommandBase setEncoderOffsetCommand() {
    return runOnce(() -> {
      resetEncoder();
      Preferences.setDouble(ArmConstants.kEncoderOffsetKey, m_encoder.getPositionOffset());  
    }).withName("Set Encoder Offset");
  }

  @Override
  public void periodic() {

      double output = m_controller.calculate(getMeasurement());
      if (enabled)
        m_motor.setVoltage(m_feedforward.calculate(m_controller.getSetpoint(), ArmConstants.kMaxVelocityRadPerSecond));
  }


}
