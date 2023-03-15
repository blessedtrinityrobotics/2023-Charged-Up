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

        public static final double kFlatGyroRoll = -11; 
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7; 

    }

    public static final class OIConstants {
        public static final int kDriverControllerId = 0;
        public static final int kOperatorControllerId = 1;
    }

    public static final class ArmConstants {
        public static final int kArmEncoderPort = 9;
        public static final int kArmMotorId = 9;
        public static final int kIntakeLeftId = 11;
        public static final int kIntakeRightId = 10; 
        public static final double kUpper = 90; // idk
        public static final double kLower = 0; 
        public static final String kEncoderOffsetKey = "EncoderOffset"; 
        public static final double kEncoderDistancePerRotation = 360; // to convert to degrees 
        public static final double kDefaultEncoderOffset = 0; 

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0; 

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class ElevatorConstants {
        public static final int kElevatorRangefinderId = 12; 
        public static final int kElevatorMotorId = 1; 

        public static final double kUpperRange = 730; // millis
        public static final double kLowerRange = 130; // mills
        // 43.5 inches tall 

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0; 

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0; 
    }

    public static final class ShuffleboardConstants {
        public static final String kArmTab = "Arm";
        public static final String kDriveTab = "Drive"; 
    }
}
