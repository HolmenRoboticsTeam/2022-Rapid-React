package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants; //imports the Constants.java variable

public class DriveSubsystem extends SubsystemBase {
  // Tank drive portion of code (The numbers assigned afterwards is a guess based
  // on what is written on the motors)
  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftFrontMotor); // 1 Includes Encoder
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightFrontMotor); // unnamed(2) Includes
                                                                                            // Encoder
  private WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(DriveConstants.kBackLeftMotor); // 0
  private WPI_VictorSPX backRightMotor = new WPI_VictorSPX(DriveConstants.kBackRightMotor); // 3

  // H-Drive portion of code
  private WPI_VictorSPX leftMotorOfH_Wheel = new WPI_VictorSPX(DriveConstants.kMiddleLeftMotor); // 3
  private WPI_VictorSPX rightMotorOfH_Wheel = new WPI_VictorSPX(DriveConstants.kMiddleRightMotor); // 2
  // left motor of H wheel would move with right motor of H wheel

  // Meter tracker I assume
  private double kEncoderTick2Meters = DriveConstants.kEncoderTick2Meter; // constant is already confirmed in
                                                                          // DriveConstants.
  private DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  public void arcadeDrive(double speedX, double speedY, double zRotation) {
    drive.arcadeDrive(speedY, zRotation);
    rightMotorOfH_Wheel.set(speedX);
  }


  // create an arcade Drive that's accesseble.

  public DriveSubsystem() {
    leftFrontMotor.configFactoryDefault();
    rightFrontMotor.configFactoryDefault();

    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);

    backLeftMotor.follow(leftFrontMotor);
    backRightMotor.follow(rightFrontMotor);

    leftMotorOfH_Wheel.follow(rightMotorOfH_Wheel);
  }

  // gets the integer that is from the DriveConstants class.
  public double getEncoderMeters() {
    return (leftFrontMotor.getSelectedSensorPosition() + -rightFrontMotor.getSelectedSensorPosition()) / 2 * kEncoderTick2Meters;
  }


  @Override
  public void periodic() {
    // get x and y button import... the moving thing on controller

    // This method will be called once per scheduler run
  }
}
