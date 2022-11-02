// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class InputConstants {
        // Controllers
        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;

        // Climber buttons
        public static final int kClimberExtendLeftJButton = 5;
        public static final int kClimberRetractLeftJButton = 3;

        // Intake buttons
        public static final int kIntakeForwardRightJButton = 5;

        // Management buttons
        public static final int kManagementForwardRightJButton = 4;
        public static final int kManagementReverseRightJButton = 6;
        public static final int kIntakeAndManagementForwardRightJButton = 3;

        // Shooter buttons
        public static final int kAutogenousFiringRightJButton = 1;
    }

    public static class ClimberConstants {
        // Motors
        public static final int kLeftMotorPort = 0;
        public static final int kRightMotorPort = 7;

        public static final NeutralMode kLeftNeutralMode = NeutralMode.Brake;
        public static final NeutralMode kRightNeutralMode = NeutralMode.Brake;

        // Limits
        public static final int kLeftLimitPort = 0;
        public static final int kRightLimitPort = 1;

        // Speeds
        public static final double kExtendSpeed = 1;
        public static final double kRetractSpeed = -1;
        public static final double kHoldSpeed = -0.15;
    }

    public static class DriveConstants {
        // Motors
        public static final int kLeftFrontMotorPort = 1;
        public static final int kRightFrontMotorPort = 2;
        public static final int kLeftBackMotorPort = 6;
        public static final int kRightBackMotorPort = 3;

        public static final InvertType kLeftFrontInverted = InvertType.None;
        public static final InvertType kRightFrontInverted = InvertType.InvertMotorOutput;
        public static final InvertType kLeftBackInverted = InvertType.FollowMaster;
        public static final InvertType kRightBackInverted = InvertType.FollowMaster;

        public static final NeutralMode kLeftFrontNeutralMode = NeutralMode.Brake;
        public static final NeutralMode kRightFrontNeutralMode = NeutralMode.Brake;
        public static final NeutralMode kLeftBackNeutralMode = NeutralMode.Brake;
        public static final NeutralMode kRightBackNeutralMode = NeutralMode.Brake;

        public static final FeedbackDevice kLeftFrontFeedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;
        public static final FeedbackDevice kRightFrontFeedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;

        public static final boolean kLeftFrontSensorPhase = true;
        public static final boolean kRightFrontSensorPhase = true;

        // Encoders
        public static final int kTicksPerRev = 4096;
        public static final double kGearRatio = 1;
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0);
        public static final double kWheelCircumferenceMeters = 2.0 * kWheelRadiusMeters * Math.PI;
    }

    public static class IntakeConstants {
        // Motors
        public static final int kIntakeMotorPort = 5;

        // Speed
        public static final double kForwardSpeed = 0.75;
    }

    public static class LimelightConstants {
        // Network tables
        public static final String kNetworkTable = "limelight";
        public static final String kXOffsetEntry = "tx";
        public static final String kYOffsetEntry = "ty";
        public static final String kTargetAreaEntry = "ta";
        public static final String kTargetFoundEntry = "tv";

        // Limelight mounting
        public static final double kLimelightMountAngle = 25.0;  // From 90 degrees vertical
        public static final double kLimelightMountHeightMeters = Units.inchesToMeters(31.25);

        // Target
        public static final double kHeightOfTargetMeters = Units.inchesToMeters(105.5);
        public static final double kHeightOfTargetFromLimelightMeters = kHeightOfTargetMeters - kLimelightMountHeightMeters;
    }

    public static class ManagementConstants {
        // Motors
        public static final int kManagementMotorPort = 4;

        // Speed
        public static final double kForwardSpeed = 1.0;
        public static final double kReverseSpeed = -0.25;
    }

    public static class ShooterAngleConstants {
        // Motors
        public static final int kShooterAngleMotorPort = 3;

        public static final NeutralMode kNeutralMode = NeutralMode.Brake;
        public static final FeedbackDevice kFeedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;
        public static final boolean kSensorPhase = false;

        // Encoders
        public static final int kTicksPerRev = 4096;
        public static final double kDegreesInCircles = 360.0;
        public static final double kPrimaryGearRatio = 200.0 / 1.0;
        public static final double kSecondaryGearRatio = 164.0 / 12.0;
        public static final double kDegreesPerPulse = kDegreesInCircles / (kPrimaryGearRatio * kSecondaryGearRatio * kTicksPerRev);

        // Motion controllers
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        // Goals and tolerances
        public static final double kGoalTolerance = 0.01;  // Decimal percentage; 1 = 100%
        public static final double kEntryAngleRadians = Units.degreesToRadians(-69);
    }

    public static class ShooterFlywheelConstants {
        // Motors
        public static final int kFrontMotorPort = 10;
        public static final int kBackMotorPort = 11;

        // DANGER: Changing the below to brake mode may cause damage
        public static final NeutralMode kFrontNeutralMode = NeutralMode.Coast;
        public static final NeutralMode kBackNeutralMode = NeutralMode.Coast;

        public static final FeedbackDevice kFrontFeedbackDevice = FeedbackDevice.IntegratedSensor;
        public static final FeedbackDevice kBackFeedbackDevice = FeedbackDevice.IntegratedSensor;

        // Motion controllers
        public static final double kFrontKP = 0.0;
        public static final double kFrontKI = 0.0;
        public static final double kFrontKD = 0.0;
        public static final double kFrontKF = 0.0;

        public static final double kBackKP = 0.0;
        public static final double kBackKI = 0.0;
        public static final double kBackKD = 0.0;
        public static final double kBackKF = 0.0;

        // Encoders
        public static int kTicksPerRev = 2048;
        public static int k100msPerSecond = 10;
        public static double kGearRatio = 4.0 / 1.0;
        public static double kWheelRadiusMeters = Units.inchesToMeters(3.0);
        public static double kWheelCircumferenceMeters = 2.0 * Math.PI * kWheelRadiusMeters;

        // Goals and tolerances
        public static final double kGoalTolerance = 0.01;  // Decimal percentage; 1 = 100%
    }

    public static class ShooterTurretConstants {
        // Motors
        public static final int kShooterTurretMotorPort = 5;

        public static final NeutralMode kNeutralMode = NeutralMode.Brake;

        public static final boolean kForwardSoftLimitEnabled = true;
        public static final boolean kReverseSoftLimitEnabled = true;
        public static final double kForwardSoftLimitThreshold = 133000;
        public static final double kReverseSoftLimitThreshold = -133000;

        public static final FeedbackDevice kFeedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;
        public static final boolean kSensorPhase = false;

        // Motion controllers
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        // Goals and tolerances
        public static final double kGoalTolerance = 1.0;  // Degrees
    }

}
