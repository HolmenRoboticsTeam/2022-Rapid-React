// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; // In progress

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterAngleSubsystem extends SubsystemBase {
  //private PIDController shooterAnglePID = new PIDController(ShooterConstants.kShooterAngleKP, ShooterConstants.kShooterAngleKI, ShooterConstants.kShooterAngleKD);
  private WPI_TalonSRX shooterAngleMotor = new WPI_TalonSRX(ShooterConstants.kVerticalShooterMotorPort);

  public ShooterAngleSubsystem() {
    shooterAngleMotor.configFactoryDefault();
    shooterAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    shooterAngleMotor.setSelectedSensorPosition(0);
  }

  public double degreesPerPulseDriveGear() {
    return (360.0 / ((164.0/12.0) * 200.0 * 4096.0));
    // Angle per rotation... with gear ratio 200-1     164-12
    // 360 / (numberOfEncoderPulsesPerRotation) * (gear ratio, 200/1, 164/12) = degrees per pulse
    // Falcon500 aka SX pulse = 2048 units per rotation
    // Talon SRX pulse = 4096
  }
  public double getCurrentAngle() {
    return getRawEncoderOutput() * degreesPerPulseDriveGear();
  }
  public void setMotor(double speed){
    shooterAngleMotor.set(speed);
  }
  public void setAngle(double angle){
    // get current angle (create a variable)
    // calculate (for PID object) 2 arguments -- current, target
    // call set motor using calculate output
    // double currentAngle = getCurrentAngle();
    // double output = shooterAnglePID.calculate(currentAngle, angle);
    // setMotor(output);
  }

  public double getRawEncoderOutput() {
    return shooterAngleMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // System.out.println(shooterAngleMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shooter Angle Encoder Raw", getRawEncoderOutput());
    SmartDashboard.putNumber("Angle", getCurrentAngle());
    
  }
}
