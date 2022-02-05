package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants; //imports the Constants.java variable
import frc.robot.Constants.OIConstants;

public class DriveSubsystem extends SubsystemBase{
  // Tank drive portion of code (The numbers assigned afterwards is a guess based on what is written on the motors)
  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftMotorPort); // 1
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightMotorPort); // unnamed(2)
  private WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(DriveConstants.kLeftEncoderChannelA); // 0
  private WPI_VictorSPX backRightMotor = new WPI_VictorSPX(DriveConstants.kRightEncoderChannelA); // 3

  // H-Drive portion of code
  private WPI_VictorSPX leftMotorOfH_Wheel = new WPI_VictorSPX(DriveConstants.kLeftEncoderChannelB); // 3
  private WPI_VictorSPX rightMotorOfH_Wheel = new WPI_VictorSPX(DriveConstants.kRightEncoderChannelB); // 2

  // Meter tracker I assume
  private double kEncoderTick2Meters = DriveConstants.kEncoderTick2Meter; // constant is already confirmed in DriveConstants.

  // Joystick?
  private Joystick stick;
  private XboxController controller;
  private int DriverJoystickPort = OIConstants.kDriverJoystickPort;
  private int ArcadeDriveSpeedAxis = OIConstants.kArcadeDriveSpeedAxis;
  private int ArcadeDriveTurnAxis = OIConstants.kArcadeDriveTurnAxis;

  // gets the integer that is from the DriveConstants class.

    public DriveSubsystem() {
    }

    @Override
    public void periodic() {
      // get x and y button import... the moving thing on controller
        stick.getRawAxis(ArcadeDriveSpeedAxis);
        stick.getRawAxis(ArcadeDriveTurnAxis); // I am struggling to find and get the x or y axis somehow
        stick.getRawAxis(DriverJoystickPort);
      // This method will be called once per scheduler run
    }
    
}
