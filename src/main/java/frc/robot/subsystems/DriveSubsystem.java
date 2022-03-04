package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants; 

public class DriveSubsystem extends SubsystemBase {

  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftFrontMotor); 
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightFrontMotor); 
                                                                                            //encoders are 
  private WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(DriveConstants.kBackLeftMotor); 
  private WPI_VictorSPX backRightMotor = new WPI_VictorSPX(DriveConstants.kBackRightMotor); 

  // H-Drive portion of code
  private WPI_VictorSPX leftMotorOfH_Wheel = new WPI_VictorSPX(DriveConstants.kMiddleLeftMotor); 
  private WPI_VictorSPX rightMotorOfH_Wheel = new WPI_VictorSPX(DriveConstants.kMiddleRightMotor); 

  private double kEncoderTick2Meters = DriveConstants.kEncoderTick2Meter; 
                                                                          
  private DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  private TalonSRXSimCollection rightMotorSim = rightFrontMotor.getSimCollection();
  private TalonSRXSimCollection leftMotorSim = leftFrontMotor.getSimCollection();

  public void arcadeDrive(double speedX, double speedY, double zRotation) {
    drive.arcadeDrive(speedY, zRotation);
    rightMotorOfH_Wheel.set(speedX);
  }

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

    leftMotorOfH_Wheel.follow(rightMotorOfH_Wheel);
  }

  public double getEncoderMeters() {
    return (leftFrontMotor.getSelectedSensorPosition() + -rightFrontMotor.getSelectedSensorPosition()) / 2 * kEncoderTick2Meters;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
  } //
}
