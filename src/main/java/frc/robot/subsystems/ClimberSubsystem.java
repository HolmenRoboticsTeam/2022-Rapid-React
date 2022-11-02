// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  // Motors
  private WPI_VictorSPX leftClimber;
  private WPI_VictorSPX rightClimber;

  // Limits
  private DigitalInput leftLimit;
  private DigitalInput rightLimit;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Motors
    this.leftClimber = new WPI_VictorSPX(ClimberConstants.kLeftMotorPort);
    this.rightClimber = new WPI_VictorSPX(ClimberConstants.kRightMotorPort);

    // Reset configurations
    this.leftClimber.configFactoryDefault();
    this.rightClimber.configFactoryDefault();

    // Set motor neutral modes
    this.leftClimber.setNeutralMode(ClimberConstants.kLeftNeutralMode);
    this.rightClimber.setNeutralMode(ClimberConstants.kRightNeutralMode);

    // Limits
    this.leftLimit = new DigitalInput(ClimberConstants.kLeftLimitPort);
    this.rightLimit = new DigitalInput(ClimberConstants.kRightLimitPort);

    // Telemetry
    SmartDashboard.putNumber("Left Climber Speed", 0.0);
    SmartDashboard.putNumber("Right Climber Speed", 0.0);
    SmartDashboard.putBoolean("Left Climber At Limit", false);
    SmartDashboard.putBoolean("Right Climber At Limit", false);
  }

  @Override
  public void periodic() {
    // Update telemetry
    SmartDashboard.putNumber("Left Climber Speed", this.leftClimber.get());
    SmartDashboard.putNumber("Right Climber Speed", this.rightClimber.get());
    SmartDashboard.putBoolean("Left Climber At Limit", this.leftLimit.get());
    SmartDashboard.putBoolean("Right Climber At Limit", this.rightLimit.get());
  }

  /**
   * Run the left climber motor at a specific speed.
   *
   * If the maximum limit switch is active, instead run the motor at
   * a hold speed.
   *
   * @param speed how fast to run the motor [-1, 1]
   */
  public void setLeftClimber(double speed) {
    // Check if the limit switch is active
    if (this.leftLimit.get()) {
      speed = ClimberConstants.kHoldSpeed;
    }

    this.leftClimber.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Run the right climber motor at a specific speed.
   *
   * If the maximum limit switch is active, instead run the motor at
   * a hold speed.
   *
   * @param speed how fast to run the motor [-1, 1]
   */
  public void setRightClimber(double speed) {
    // Check if the limit switch is active
    if (this.rightLimit.get()) {
      speed = ClimberConstants.kHoldSpeed;
    }

    this.rightClimber.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stop both climber motors.
   */
  public void stop() {
    this.leftClimber.stopMotor();
    this.rightClimber.stopMotor();
  }

}
