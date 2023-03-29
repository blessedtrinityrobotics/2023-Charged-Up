package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class DriveConstants {
        public static final int kFrontRightDriveId = 4;
        public static final int kBackRightDriveId = 5;
        public static final boolean kRightInverted = true; 
        public static final int kFrontLeftDriveId = 6;
        public static final int kBackLeftDriveId = 7;
        public static final boolean kLeftInverted = false; 
        public static final int kPigeonId = 3;

        public static final int[] kLeftEncoderPorts = {0, 1};
        public static final boolean kLeftEncoderReversed = false;
        public static final int[] kRightEncoderPorts = {2, 3};
        public static final boolean kRightEncoderReversed = true;
        public static final double kWheelDiameter = 0.1524; // Wheels are 6 inches --> to meters
        public static final double kEncoderCPR = 2048; // Cycles (NOT COUNTS) per revolution, we are using Rev Through Bore Encoders 
        public static final double kDistancePerPulse = 1.0 / kEncoderCPR * kWheelDiameter * Math.PI; // For every one rotation, distance traveled should be the circumference of wheel

        public static final double ksVolts = 0.049852;
        public static final double kvVoltSecondsPerMeter = 7.7964;
        public static final double kaVoltSecondsSquaredPerMeter = 1.3909;

        public static final double empiricalTrackWidthMeters = 0.24251; // meters 
        public static final double physicalTrackWidthMeters = 0.5715; // meters 
        public static final double kPDriveVel = 3.3015; // P gain for feedback 
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(physicalTrackWidthMeters);

        // offset for gyro when it is horizontal 
        public static final double kFlatGyroRoll = -11; 
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        
        // For the balance auto 
        public static final double kRollBackDegrees = 13;
        public static final double kBalancedDegrees = 10;
    }

    public static final class OIConstants {
        public static final int kDriverControllerId = 0;
        public static final int kOperatorControllerId = 1;
        public static final double kJoystickDeadzone = 0.05; 
        public static final int kDegreeSmoothing = 2; // what the joystick input is raised to 
        public static final double kArmSensitivity = 0.01; // how reactive the setpoint is 
        public static final double kElevatorSensitivity = 0.01; // how reactive the setpoint changes are 

        public static final double kBaselinePower = 0.3; // where motors should start at in Teleop
        public static final double kMaxPower = 1.0; // max power that can be given to motors in teleop 
    }

    public static final class ArmConstants {
        public static final int kArmEncoderPort = 9;
        public static final int kArmMotorId = 1;
        public static final int kIntakeLeftId = 11;
        public static final int kIntakeRightId = 10; 
        public static final double kUpper = Math.PI / 2.1; 
        public static final double kLower = 0; 
        public static final double kHighPosition = Math.PI / 4; // make sure this is between lower and upper 
        public static final double kDefaultEncoderOffset = 0; 
        public static final String kEncoderOffsetKey = "EncoderOffset"; 
        public static final double kEncoderDistancePerRotation = 6.28;  
        public static final double kMaxPower = 1.0;
        public static final double kMinPower = -0.5;

        public static final double kP = 0.8;
        public static final double kI = 0; // leave this at zero for your safety 
        public static final double kD = 0;
        public static final double kMaxVelocityRadPerSecond = 5;
        public static final double kMaxAccelerationRadPerSecSquared = 3;
    }

    public static final class ElevatorConstants {
        public static final int kElevatorRangefinderId = 15; 
        public static final int kElevatorMotorId = 12; 

        public static final double kErrorTolerance = 0.02; // m
        public static final double kUpper = 0.73; // m
        public static final double kMidCubePos = 0.6; 
        public static final double kLower = 0.10; // m

        public static final double kP = 5;
        public static final double kI = 0; // leave this at zero for your safety 
        public static final double kD = 0;
        public static int kMinPower = -1;
        public static int kMaxPower = 1; 

        public static double kElevatorFeedforward = 0.075; // initial power offset 
        public static double kMaxVelocityMetersPerSecond = 4;
        public static double kMaxAccelerationMetersPerSecondSquared = 3; 
    }

    public static final class ShuffleboardConstants {
        public static final String kArmTab = "Arm";
        public static final String kDriveTab = "Drive"; 
        public static final String kAutoTab = "Auto";
        public static final String kIntakeTab = "Intake";
        public static final String kElevatorTab = "Elevator"; 
    }

    public static final class IntakeConstants {
        public static final double kDefaultOutakeSpeed = 1; 
        public static final double kDefaultIntakeSpeed = -0.2; 
    }
}
