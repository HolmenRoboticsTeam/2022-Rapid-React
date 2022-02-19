// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // desired motor?

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterRotationSubsystem extends SubsystemBase {
  private Spark shooterRotationMotor = new Spark(ShooterConstants.kShooterRotationMotorPort);

  /** Creates a new ShooterRotationSubsystem. */

  public ShooterRotationSubsystem() {

  }
  public void ShooterRotation(double speed){
    shooterRotationMotor.set(speed);
  }

  public void RotationToggle(boolean limelightOn) {
    if (limelightOn) {
      shooterRotationMotor.set(ShooterConstants.shooterRotationRunSpeed);
      System.out.println(ShooterConstants.shooterRotationRunSpeed);
    } else {
      shooterRotationMotor.set(0); // :)
      System.out.println("shooter set to zero");
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}