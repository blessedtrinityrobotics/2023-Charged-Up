// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class Arm extends SubsystemBase {
  DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort); 

  WPI_TalonFX m_motor = new WPI_TalonFX(ArmConstants.kArmMotorId); 
  ProfiledPIDController m_controller = new ProfiledPIDController(
    ArmConstants.kP,ArmConstants.kI,ArmConstants.kD, 
    new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared));
  ShuffleboardTab armTab; 
  boolean enabled = false;
  public boolean manualOverride = false;
  double output = 0.0;  

  /** Creates a new Arm. */
  public Arm() {
    m_motor.setNeutralMode(NeutralMode.Brake);

    configureEncoder();
    configureArmTab();
    disablePID();    

    m_controller.setTolerance(ArmConstants.kErrorTolerance);
    m_controller.setGoal(ArmConstants.kUpper);
  }

  private void configureEncoder() {
    Preferences.initDouble(ArmConstants.kEncoderOffsetKey, ArmConstants.kDefaultEncoderOffset);

    m_encoder.setDistancePerRotation(ArmConstants.kEncoderDistancePerRotation);
    resetEncoder();
    m_encoder.setPositionOffset(Preferences.getDouble(ArmConstants.kEncoderOffsetKey, ArmConstants.kDefaultEncoderOffset));
  }


  private void configureArmTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab(ShuffleboardConstants.kDriveTab);
    driveTab.addBoolean("Manual Arm Override", () -> manualOverride); 
    driveTab.addDouble("Arm Angle", () -> getMeasurement()); 

    armTab = Shuffleboard.getTab(ShuffleboardConstants.kArmTab);
    armTab.add("Zero Command", setEncoderOffsetCommand()).withSize(2, 1);
    armTab.addDouble("Arm Angle", () -> Math.toDegrees(getMeasurement()))
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", Math.toDegrees(ArmConstants.kLower), "max", Math.toDegrees(ArmConstants.kUpper)));
    armTab.addDouble("Absolute Angle", m_encoder::getAbsolutePosition);
    armTab.add("PID", m_controller).withSize(1, 2);

    ShuffleboardLayout pidStuff = armTab.getLayout("Extra PID Info", BuiltInLayouts.kList).withSize(2, 3);

    pidStuff.addDouble("Error", m_controller::getPositionError);
    pidStuff.addBoolean("At setpoint", this::actuallyAtGoal);
    pidStuff.addDouble("PID Output", () -> output); 
    pidStuff.addDouble("Arm power", m_motor::get);

  }

  /**
   * Created because the m_controller.atGoal() method doesn't seem to work
   * Super easy to implement, and this does the trick!
   */
  private boolean actuallyAtGoal() {
    return Math.abs(m_controller.getPositionError()) < ArmConstants.kErrorTolerance; 
  }

  public double getMeasurement() {
    return m_encoder.getDistance();  
  }

  public void enableOverride() {
    manualOverride = true; 
    disablePID();
  }

  public void disableOverride() {
    enablePID();
    manualOverride = false; 
  }

  public CommandBase setSetpointCommand(DoubleSupplier direction) {
      return run(() -> {
        double newSetpoint = MathUtil.clamp(m_controller.getGoal().position + direction.getAsDouble(), 
          ArmConstants.kLower, ArmConstants.kUpper);
        m_controller.setGoal(newSetpoint);
      }); 
  }

  public void enablePID() {
    m_controller.reset(getMeasurement());
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
      // double power = direction.getAsDouble(); 
      // if (m_encoder.getDistance() < ArmConstants.kLower) 
      //   power = Math.max(0, power);
      // else if (m_encoder.getDistance() > ArmConstants.kUpper)
      //   power = Math.min(0, power);

      m_motor.set(ControlMode.PercentOutput, MathUtil.clamp(direction.getAsDouble(), -0.2, 0.3));
    });
  }

  public Command retractedCommand() {
    return runOnce(() -> m_controller.setGoal(ArmConstants.kUpper))
      .until(this::actuallyAtGoal); 
  }

  public Command highCubeCommand() {
    return run(() -> m_controller.setGoal(ArmConstants.kHighPosition))
      .until(this::actuallyAtGoal); 
  }

  public Command humanStationCommand() {
    return run(() -> m_controller.setGoal(ArmConstants.kHumanPosition))
      .until(this::actuallyAtGoal);  
  }

  public Command midConeCommand() {
    return run(() -> m_controller.setGoal(ArmConstants.kMidConePosition))
      .until(this::actuallyAtGoal); 
  }

  public Command horizontalCommand() {
    return runOnce(() -> m_controller.setGoal(ArmConstants.kLower))
      .until(this::actuallyAtGoal); 
  }

  public Command midCubeCommand() {
    return runOnce(() -> m_controller.setGoal(ArmConstants.kMidCubePosition))
      .until(this::actuallyAtGoal);
  }

  /**
   * To use this, move the arm to the zero position, and click the button 
   */
  public CommandBase setEncoderOffsetCommand() {
    return runOnce(() -> {
      resetEncoder();
      Preferences.setDouble(ArmConstants.kEncoderOffsetKey, m_encoder.getPositionOffset());
    }).withName("Set Encoder Zero");
  }

  @Override
  public void periodic() {
    output = MathUtil.clamp(m_controller.calculate(getMeasurement()), ArmConstants.kMinPower, ArmConstants.kMaxPower); 
    if (enabled)
      m_motor.set(ControlMode.PercentOutput, output); 
  }


}
