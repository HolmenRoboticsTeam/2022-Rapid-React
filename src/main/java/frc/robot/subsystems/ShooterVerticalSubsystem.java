// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; // In progress

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterVerticalSubsystem extends SubsystemBase {
  private WPI_TalonSRX shooterVerticalSubsystem = new WPI_TalonSRX(ShooterConstants.kVerticalShooterMotorPort);

  public ShooterVerticalSubsystem() {

  }

  public void ShooterVerticalMovement(double speed){
    shooterVerticalSubsystem.set(speed * ShooterConstants.verticalMotorRunSpeed);
  }

  @Override
  public void periodic() {} //
}
