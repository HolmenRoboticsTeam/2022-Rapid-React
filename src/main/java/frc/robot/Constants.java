package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class magamentconstants{
        public static final int magamentoterID = 0;
        public static final double managerrunspeed = 1;
        public static final double magegerfullstop = 0;
    }

    public static final class JoystickDrive {
        //public
    }

    public static final class DriveConstants {
        public static final int kLeftMotorPort = 0; // Talons I'd assume
        public static final int kRightMotorPort = 1; // The unmarked motor I'd assume
        public static final int kLeftEncoderChannelA = 0; //  Victor for Tank Drive -BackLeft?
        public static final int kLeftEncoderChannelB = 1; // Victor for H-Drive - left?
        public static final int kRightEncoderChannelA = 2; // Victor for Tank Drive -BackRight?
        public static final int kRightEncoderChannelB = 3; // Victor for H-Drive - right?
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;
// autonomus
        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 1.5;
    }

    public static final class ElevatorConstants {
        public static final int kMotorPort = 2;
        public static final int kEncoderChannelA = 4;
        public static final int kEncoderChannelB = 5;
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;
        public static final double kP = 3;
        public static final double kI = 0;
        public static final double kD = 0.8;

        public static final double kRaisedPosition = 1.2;
        public static final double kLoweredPosition = 0;
        public static final double kJoystickMaxSpeed = 0.5;
    }

    public static final class IntakeConstants {
        public static final int kLeftMotorPort = 3;
        public static final int kRightMotorPort = 4;
    }

    public static final class ManagementConstants {
        public static final int kMainMotorPort = 3;
    }

    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;

        public static final int kRotateJoystickPort = 1;

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 3;

        public static final int kElevatorPIDRaiseButtonIdx = 1;
        public static final int kElevatorPIDLowerButtonIdx = 2;
        public static final int kElevatorJoystickRaiseButtonIdx = 3;
        public static final int kElevatorJoystickLowerButtonIdx = 4;

        public static final int kIntakeButtonIdx = 5;

        public static final int kManagementButtonIdx = 6;

    }
}
