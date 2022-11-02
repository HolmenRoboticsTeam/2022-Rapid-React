// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Motors
  private WPI_TalonSRX leftFrontMotor;
  private WPI_TalonSRX rightFrontMotor;
  private WPI_VictorSPX leftBackMotor;
  private WPI_VictorSPX rightBackMotor;

  // Drivebase
  private DifferentialDrive drive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Motors
    this.leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
    this.rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
    this.leftBackMotor = new WPI_VictorSPX(DriveConstants.kLeftBackMotorPort);
    this.rightBackMotor = new WPI_VictorSPX(DriveConstants.kRightBackMotorPort);

    // Reset motor configs
    this.leftFrontMotor.configFactoryDefault();
    this.rightFrontMotor.configFactoryDefault();
    this.leftBackMotor.configFactoryDefault();
    this.rightBackMotor.configFactoryDefault();

    // Set followers
    this.leftBackMotor.follow(this.leftFrontMotor);
    this.rightBackMotor.follow(this.rightFrontMotor);

    // Set inverts
    this.leftFrontMotor.setInverted(DriveConstants.kLeftFrontInverted);
    this.rightFrontMotor.setInverted(DriveConstants.kRightFrontInverted);
    this.leftBackMotor.setInverted(DriveConstants.kLeftBackInverted);
    this.rightBackMotor.setInverted(DriveConstants.kRightBackInverted);

    // Set neutral modes
    this.leftFrontMotor.setNeutralMode(DriveConstants.kLeftFrontNeutralMode);
    this.rightFrontMotor.setNeutralMode(DriveConstants.kRightFrontNeutralMode);
    this.leftBackMotor.setNeutralMode(DriveConstants.kLeftBackNeutralMode);
    this.rightBackMotor.setNeutralMode(DriveConstants.kRightBackNeutralMode);

    // Set feedback sensors
    this.leftFrontMotor.configSelectedFeedbackSensor(DriveConstants.kLeftFrontFeedbackDevice);
    this.rightFrontMotor.configSelectedFeedbackSensor(DriveConstants.kRightFrontFeedbackDevice);

    // Set sensor phases
    this.leftFrontMotor.setSensorPhase(DriveConstants.kLeftFrontSensorPhase);
    this.rightFrontMotor.setSensorPhase(DriveConstants.kRightFrontSensorPhase);

    // Reset encoder positions
    this.leftFrontMotor.setSelectedSensorPosition(0);
    this.rightFrontMotor.setSelectedSensorPosition(0);

    // Drivebase
    this.drive = new DifferentialDrive(this.leftFrontMotor, this.rightFrontMotor);

    // Telemetry
    SmartDashboard.putNumber("Drive Train Left Speed", 0);
    SmartDashboard.putNumber("Drive Train Right Speed", 0);
  }

  @Override
  public void periodic() {
    // Update telemetry
    SmartDashboard.putNumber("Drive Train Left Speed", this.leftFrontMotor.get());
    SmartDashboard.putNumber("Drive Train Right Speed", this.rightFrontMotor.get());
  }

  /**
   * Drive the robot forward and/or with turn.
   *
   * Remember to invert the throttle before sending it to this method!
   *
   * @param throttle how fast to send the robot forward/backward.
   * @param turn how fast to turn the robot.
   */
  public void arcadeDrive(double throttle, double turn) {
    this.drive.arcadeDrive(throttle, turn);
  }

  /**
   * Stop the drivebase.
   */
  public void stop() {
    this.drive.stopMotor();
  }

  /**
   * Get how far the drivebase has traveled.
   *
   * @return The distance traveled of the drivebase
   * averaged between the left and right tracks.
   */
  public double distanceTraveled() {
    double leftDistance = this.nativeUnitsToDistanceMeters(this.leftFrontMotor.getSelectedSensorPosition());
    double rightDistance = this.nativeUnitsToDistanceMeters(this.rightFrontMotor.getSelectedSensorPosition());
    return (leftDistance + rightDistance) / 2.0;
  }

  /**
   * Calculate how far a wheel has traveled in meters.
   *
   * @param counts raw sensor counts.
   * @return How far a wheel has traveled in meters.
   */
  private double nativeUnitsToDistanceMeters(double counts) {
    double motorRotations = counts / DriveConstants.kTicksPerRev;
    double wheelRotations = motorRotations / DriveConstants.kGearRatio;
    double positionMeters = wheelRotations * DriveConstants.kWheelCircumferenceMeters;
    return positionMeters;
  }

}
