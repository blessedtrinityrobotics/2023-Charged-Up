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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class Arm extends SubsystemBase {
  DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort); 

  WPI_TalonSRX m_arm = new WPI_TalonSRX(ArmConstants.kArmMotorId); 
  
  ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
  PIDController pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

  ShuffleboardTab m_armTab; 
  double encoderOffset = 0; 

  /** Creates a new Arm. */
  public Arm() {
    Preferences.initDouble(ArmConstants.kEncoderOffsetKey, ArmConstants.kDefaultEncoderOffset);

    m_arm.setInverted(true);
    m_armEncoder.setDistancePerRotation(ArmConstants.kEncoderDistancePerRotation);
    m_armEncoder.setPositionOffset(Preferences.getDouble(ArmConstants.kEncoderOffsetKey, ArmConstants.kDefaultEncoderOffset));

    m_arm.setNeutralMode(NeutralMode.Brake);

    m_armTab = Shuffleboard.getTab(ShuffleboardConstants.kArmTab);
    m_armTab.add("Set Encoder Offset", setEncoderOffsetCommand()).withSize(2, 1);
    m_armTab.addDouble("Arm Angle", () -> m_armEncoder.getDistance())
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", ArmConstants.kLower, "max", ArmConstants.kUpper)); 
  }


  public Command moveArmCommand(DoubleSupplier direction) {
    return run(() -> {
      double power = direction.getAsDouble(); 
      if (m_armEncoder.getDistance() < ArmConstants.kLower) 
        power = Math.max(0, power);
      else if (m_armEncoder.getDistance() > ArmConstants.kUpper)
        power = Math.min(0, power);

      m_arm.set(ControlMode.PercentOutput, power);
    });
  }

  public void resetEncoder() {
    m_armEncoder.reset();
  }

  public CommandBase setEncoderOffsetCommand() {
    return runOnce(() -> {
      resetEncoder();
      Preferences.setDouble(ArmConstants.kEncoderOffsetKey, m_armEncoder.getPositionOffset());  
    }).withName("Set Encoder Offset");
  }

  @Override
  public void periodic() {
  }
}
