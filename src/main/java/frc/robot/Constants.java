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

    public static final class DriveConstants {
        public static final int kLeftFrontMotor = 2; // Talons (gets feedback)              Talon 2
        public static final int kRightFrontMotor = 1; // The unmarked motor                 Talon 1
        public static final int kBackLeftMotor = 3; // for the tank                         Victor 3
        public static final int kMiddleLeftMotor = 2; // for the H-drive                     Victor 2
        public static final int kBackRightMotor = 0; // for the tank                        Victor 0
        public static final int kMiddleRightMotor = 1; // for the H-drive                   Victor 1
        public static final double kEncoderTick2Meter = 4096.0 * 1.0 * 0.2032 * Math.PI;
// autonomus
        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 1.5;
    }

    public static final class IntakeConstants {
        public static final int kLeftMotorPort = 5; //                  Victor 5
        public static final int kRightMotorPort = 11; // uneeded

        public static final double kSpeed = 0.5;
    }

    public static final class ClimberConstants {
        public static final int kMainMotorPort = 11;

        public static final int kSpeed = 1;
    }

    public static final class ManagementConstants {
        public static final int kManagementMotorPort = 4; //            Victor 4
// get intake and management to run together
        public static final double kSpeed = 1;
        public static final double kSpeedAlt = -0.25;
    }

    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0; //left joystick

        public static final int kDriverJoystickPort2 = 1; //right joystick

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 3;

        public static final int kIntakeButtonIdx = 5; // left middle button: RIGHT JOYSTICK

        public static final int kManagementButtonIdx = 4; // Goes Fast
        public static final int kManagementButton2 = 6; // Goes Slow
        public static final int kManagementAndIntakeIdx = 3; // Has both intake and management run at the same time

        public static final int kShooterButtonIdx = 1; // right trigger: RIGHT JOYSTICK (parallels with the management)

        public static final int kClimberButtonUpIdx = 6; // right middle button: LEFT JOYSTICK
        public static final int kClimberButtonDownIdx = 4; // right bottom button: LEFT JOYSTICK

        public static final int kRotationButtonIdx = 11; // for vertical shooter

        // public static final int kElevatorPIDRaiseButtonIdx = 1;
        // public static final int kElevatorPIDLowerButtonIdx = 2;
        // public static final int kElevatorJoystickRaiseButtonIdx = 3;
        // public static final int kElevatorJoystickLowerButtonIdx = 4;
    }

    public static final class ShooterConstants {
        public static final int kShootertMotorPort = 11;
        public static final double shooterRunSpeed = 1;
        public static final double shooterStopSpeed = 0;

        public static final int kShooterRotationMotorPort = 3; // elevator motor
        public static final int kVerticalShooterMotorPort = 215;

        public static final double shooterRotationRunSpeed = 0.05;
        public static final double verticalMotorRunSpeed = 2;

        public static final double rotationConstantAngle = 2.5;
        public static final double kMinimumRotationSpeed = -0.4;
        public static final double kMaximumRotationSpeed = 0.4;

        public static final double kLimeLightAngle = 15; // degrees
        public static final double kLimeLightHeightFromGround = 5.5; // ft
        public static final double kHeightOfTarget = 8.8; // ft
        public static final double kPControlConstant = -0.1;
    }

    //          /* currently not being used */
//     public static final class ElevatorConstants {
//         public static final int kMotorPort = 2;
//         public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;
//         public static final double kP = 3;
//         public static final double kI = 0;
//         public static final double kD = 0.8;

//         public static final double kRaisedPosition = 1.2;
//         public static final double kLoweredPosition = 0;
//         public static final double kJoystickMaxSpeed = 0.5;
//     }

}
