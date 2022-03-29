// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterRotationSubsystem extends SubsystemBase {
  private WPI_TalonSRX shooterRotationMotor = new WPI_TalonSRX(ShooterConstants.kShooterRotationMotorPort);
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private boolean atMaximumBound = false;
  private boolean atMinimumBound = false;

  public ShooterRotationSubsystem() {
    shooterRotationMotor.configFactoryDefault();
    shooterRotationMotor.setNeutralMode(NeutralMode.Brake);  // Try to maintain the position when stopped
    shooterRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    shooterRotationMotor.setSensorPhase(true);  // If CW(clockwise) rotation is decreasing encoder, set to false
    shooterRotationMotor.setSelectedSensorPosition(0);
    gyro.reset();
  }

  public void setMotor(double speed){
    // Prevent the turret from exceeding its limits
    double encoderRaw = shooterRotationMotor.getSelectedSensorPosition();
    boolean encoderIncreasing = speed > 0;

    // if (encoderRaw >= ShooterConstants.kMaximumEncoderValue && encoderIncreasing) {
    //   shooterRotationMotor.set(0);
    //   atMaximumBound = true;
    //   return;
    // } else if (encoderRaw <= ShooterConstants.kMinimumEncoderValue && !encoderIncreasing) {
    //   shooterRotationMotor.set(0);
    //   atMinimumBound = true;
    //   return;
    // }

    // Allow motor to run
    atMinimumBound = false;
    atMaximumBound = false;
    shooterRotationMotor.set(speed);
  }

  public double getGyroHeading() {
    return gyro.getAngle();
  }

  public boolean isAtMaximumBound() {
    return atMaximumBound;
  }

  public boolean isAtMinimumBound() {
    return atMinimumBound;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Rotation Raw", shooterRotationMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shooter Rotation Speed", shooterRotationMotor.get());
  }
}
