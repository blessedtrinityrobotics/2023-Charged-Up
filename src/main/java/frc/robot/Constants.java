package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;

public final class Constants {
    public static final class DriveConstants {
        public static final int kFrontRightDriveId = 4;
        public static final int kBackRightDriveId = 5;
        public static final int kFrontLeftDriveId = 6;
        public static final int kBackLeftDriveId = 7;
        public static final int kPigeonId = 0;
        public static final int[] kLeftEncoderPorts = {0, 1};
        public static final boolean kLeftEncoderReversed = false;
        public static final int[] kRightEncoderPorts = {2, 3};
        public static final boolean kRightEncoderReversed = true;
    }

    public static final class OIConstants {
        public static final int kDriverControllerId = 0;
    }
}
