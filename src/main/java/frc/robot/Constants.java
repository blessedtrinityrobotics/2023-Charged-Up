package frc.robot;

public final class Constants {
    public static final class DriveConstants {
        public static final int kFrontRightDriveId = 4;
        public static final int kBackRightDriveId = 5;
        public static final boolean kRightInverted = true; 
        public static final int kFrontLeftDriveId = 6;
        public static final int kBackLeftDriveId = 7;
        public static final boolean kLeftInverted = false; 
        public static final int kPigeonId = 0;
        public static final int[] kLeftEncoderPorts = {0, 1};
        public static final boolean kLeftEncoderReversed = false;
        public static final int[] kRightEncoderPorts = {2, 3};
        public static final boolean kRightEncoderReversed = true;
        public static final double kWheelDiameter = 0.1524; // Wheels are 6 inches --> to meters
        public static final double kEncoderCPR = 8192; // Counts per revolution, we are using Rev Through Bore Encoders 
        public static final double kDistancePerPulse = 1.0 / kEncoderCPR * kWheelDiameter * Math.PI; // For every one rotation, distance traveled should be the circumference of wheel
    }

    public static final class OIConstants {
        public static final int kDriverControllerId = 0;
    }
}
