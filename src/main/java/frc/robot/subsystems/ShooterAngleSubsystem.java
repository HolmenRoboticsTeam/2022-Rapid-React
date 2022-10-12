// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; // In progress

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterAngleSubsystem extends SubsystemBase {
  private WPI_TalonSRX shooterAngleMotor = new WPI_TalonSRX(ShooterConstants.kVerticalShooterMotorPort);

  public ShooterAngleSubsystem() {
    shooterAngleMotor.configFactoryDefault();
    shooterAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    shooterAngleMotor.setSelectedSensorPosition(0);
  }

  public void setMotor(double speed){
    shooterAngleMotor.set(speed);
  }

  public double getRawEncoderOutput() {
    return shooterAngleMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // System.out.println(shooterAngleMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shooter Angle Encoder Raw", getRawEncoderOutput());
    
  }
}
