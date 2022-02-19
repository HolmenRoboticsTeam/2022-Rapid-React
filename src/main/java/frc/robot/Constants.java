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

    public static final class JoystickDrive {
        //public
    }

    public static final class DriveConstants {
        public static final int kLeftFrontMotor = 1; // Talons (gets feedback)
        public static final int kRightFrontMotor = 2; // The unmarked motor
        public static final int kBackLeftMotor = 0; // for the tank
        public static final int kMiddleLeftMotor = 3; // for the H-drive
        public static final int kBackRightMotor = 3; // for the tank
        public static final int kMiddleRightMotor = 2; // for the H-drive
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;
// autonomus
        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 1.5;
    }

    public static final class ElevatorConstants {
        public static final int kMotorPort = 2;
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

        public static final int kSpeed = 1;
    }

    public static final class ClimberConstants {
        public static final int kMainMotorPort = 8;

        public static final int kSpeed = 1;
    }

    public static final class ManagementConstants {
        public static final int kManagementMotorPort = 0;

        public static final int kSpeed = 1;
    }

    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;

        public static final int kDriverJoystickPort2 = 1;

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 3;

        // public static final int kElevatorPIDRaiseButtonIdx = 1;
        // public static final int kElevatorPIDLowerButtonIdx = 2;
        // public static final int kElevatorJoystickRaiseButtonIdx = 3;
        // public static final int kElevatorJoystickLowerButtonIdx = 4;

        public static final int kIntakeButtonIdx = 5; // left middle button: RIGHT JOYSTICK

        public static final int kManagementButtonIdx = 3; // left bottom button: RIGHT JOYSTICK

        public static final int kShooterButtonIdx = 1; // right trigger: RIGHT JOYSTICK (parallels with the management)

        public static final int kClimberButtonUpIdx = 6; // right middle button: LEFT JOYSTICK
        public static final int kClimberButtonDownIdx = 4; // right bottom button: LEFT JOYSTICK

        public static final int kRotationButtonIdx = 99;


    }

    public static final class ShooterConstants {
        public static final int kShootertMotorPort = 0;
        public static final double shooterRunSpeed = 1;
        public static final double shooterStopSpeed = 0;

        public static final int kShooterRotationMotorPort = 7;
        public static final int kVerticalShooterMotorPort = 215;

        public static final double shooterRotationRunSpeed = 0.05;
        public static final double verticalMotorRunSpeed = 2;

        public static final double rotationConstantAngle = 2.5;
        public static final double kMinimumRotationSpeed = 0.0;
        public static final double kMaximumRotationSpeed = 0.2;

        public static final double kLimeLightAngle = 15; // degrees
        public static final double kLimeLightHeightFromGround = 5.5; // ft //66 inches
        public static final double kHeightOfTarget = 8.8; // ft
        public static final double kPControlConstant = -0.1;
    }

    public static final class ButtonConstants {
        public static final int buttonnn = 0;
    }
}
