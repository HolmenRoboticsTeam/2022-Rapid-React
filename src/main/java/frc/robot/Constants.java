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
        public static final int kLeftFrontMotor = 1; // Talons (gets feedback)              Talon 1
        public static final int kRightFrontMotor = 2; // The unmarked motor                 Talon 2
        public static final int kBackLeftMotor = 6; // for the tank                         Victor 6
        //public static final int kMiddleLeftMotor = 1; // for the H-drive                     Victor 1
        public static final int kBackRightMotor = 3; // for the tank                        Victor 3
        //public static final int kMiddleRightMotor = 2; // for the H-drive                   Victor 2
        public static final double kEncoderTick2Meter = 4096.0 * 1.0 * 0.2032 * Math.PI;

        // autonomus
        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 1.5;
    }

    public static final class IntakeConstants {
        public static final int kLeftMotorPort = 5; //                  Victor 5

        public static final double kSpeed = 0.75;
    }

    public static final class ClimberConstants {
        public static final int kLeftClimberMotorPort = 0;
        public static final int kRightClimberMotorPort = 7;

        public static final int kRightDigitalInput = 0;
        public static final int kLeftDigitalInput = 1;

        public static final double kExtendSpeed = 1;
        public static final double kRetractSpeed = -0.8;

        public static final double kHoldSpeed = -0.15;
    }

    public static final class ManagementConstants {
        public static final int kManagementMotorPort = 4; //            Victor 4

        public static final double kSpeed = 1;
        public static final double kSpeedAlt = -0.25;
    }

    public static final class OIConstants {
        public static final int kLeftHandedJoystickPort = 0; //left joystick

        public static final int kRightHandedJoystickPort = 1; //right joystick

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 3;

        public static final int kIntakeButtonIdx = 5; // left middle button: RIGHT JOYSTICK

        public static final int kManagementButtonIdx = 4; // Goes Fast
        public static final int kManagementButton2 = 6; // Goes backwards
        public static final int kManagementAndIntakeIdx = 3; // Has both intake and management run at the same time

        public static final int kShooterButtonIdx = 1; // right trigger: RIGHT JOYSTICK (parallels with the management)

        public static final int kClimberButtonUpIdx = 5; // right middle button: LEFT JOYSTICK
        public static final int kClimberButtonDownIdx = 3; // right bottom button: LEFT JOYSTICK

        public static final int kRotationButtonIdx = 11; // for vertical shooter
    }

    public static final class ShooterConstants {
        public static final int kShooterFlywheelFrontMotorPort = 0;
        public static final int kShooterFlywheelBackMotorPort = 4;

        public static final double kShooterRunSpeed = -0.75;
        public static final double kShooterAutoRunSpeed = -0.5;
        public static final double kShooterStopSpeed = 0;

        public static final int kShooterRotationMotorPort = 5;
        public static final int kVerticalShooterMotorPort = 3;

        public static final double shooterRotationRunSpeed = 0.05;
        public static final double verticalMotorRunSpeed = 2;

        public static final double rotationConstantAngle = 2.5;
        public static final double kMinimumRotationSpeed = -0.1;
        public static final double kMaximumRotationSpeed = 0.1;
        public static final double kMinimumEncoderValue = -75000;  // TODO: Determine manually
        public static final double kMaximumEncoderValue = 75000;  // TODO: Determine manually

        public static final double kLimelightMountAngle = 60; // degrees
        public static final double kLimeLightHeightFromGroundMeters = 0.813;
        public static final double kHeightOfTargetMeters = 1.854;
        public static final double kPControlConstant = -0.1;

        public static final double kHoldRotationSpeed = 0;

        public static final double kGearDiameter = 0.31;
        public static final double kConstantGearRatio = 129.23;
        public static final double kMaxRPM = 0;
        public static final double kUnitsPerRotation = 0;
    }
}
