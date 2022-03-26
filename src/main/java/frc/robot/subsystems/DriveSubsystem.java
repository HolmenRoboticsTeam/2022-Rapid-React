package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftFrontMotor);
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightFrontMotor);
  private WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(DriveConstants.kBackLeftMotor);
  private WPI_VictorSPX backRightMotor = new WPI_VictorSPX(DriveConstants.kBackRightMotor);

  // H-Drive portion of code
  private WPI_VictorSPX hWheelLeftMotor = new WPI_VictorSPX(DriveConstants.kMiddleLeftMotor);
  private WPI_VictorSPX hWheelRightMotor = new WPI_VictorSPX(DriveConstants.kMiddleRightMotor);

  private DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  // Brake mode (the motor attempts to slow electrically) reduces wheel slip
  private final NeutralMode motorMode = NeutralMode.Brake;

  public DriveSubsystem() {
    leftFrontMotor.configFactoryDefault();
    rightFrontMotor.configFactoryDefault();

    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);

    rightFrontMotor.setInverted(true);
    backRightMotor.setInverted(true);

    leftFrontMotor.setSensorPhase(true);
    rightFrontMotor.setSensorPhase(true);

    backLeftMotor.follow(leftFrontMotor);
    backRightMotor.follow(rightFrontMotor);

    leftFrontMotor.setNeutralMode(motorMode);
    rightFrontMotor.setNeutralMode(motorMode);
    backLeftMotor.setNeutralMode(motorMode);
    backRightMotor.setNeutralMode(motorMode);

    leftFrontMotor.configNeutralDeadband(0.001);
    rightFrontMotor.configNeutralDeadband(0.001);
    backLeftMotor.configNeutralDeadband(0.001);
    backRightMotor.configNeutralDeadband(0.001);

    hWheelLeftMotor.follow(hWheelRightMotor);
  }

  public void arcadeDrive(double speedX, double speedY, double zRotation) {
    drive.arcadeDrive(speedY, zRotation);
    hWheelRightMotor.set(speedX);
  }

  public double getEncoderMeters() {
    return (leftFrontMotor.getSelectedSensorPosition() + rightFrontMotor.getSelectedSensorPosition() / 2.0) / 4096.0 * 1.0 * (0.2032 * Math.PI) * (2.0/3.0);
    // raw value / 4096 * gear ratio * (diameter * pi)
    // diameter = 12.051
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive Train Left Speed", leftFrontMotor.get());
    SmartDashboard.putNumber("Drive Train Right Speed", rightFrontMotor.get());
    SmartDashboard.putNumber("Drive Train H-Wheel Speed", hWheelRightMotor.get());
  }
}
