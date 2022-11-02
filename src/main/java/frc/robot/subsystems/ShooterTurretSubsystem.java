// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterTurretConstants;

public class ShooterTurretSubsystem extends SubsystemBase {
  // Motors
  private WPI_TalonSRX shooterTurretMotor;

  // Motion controllers
  private PIDController feedbackController;

  // Offset
  private double offset;

  /** Creates a new ShooterTurretSubsystem. */
  public ShooterTurretSubsystem() {
    // Motors
    this.shooterTurretMotor = new WPI_TalonSRX(ShooterTurretConstants.kShooterTurretMotorPort);

    // Reset motor configs
    this.shooterTurretMotor.configFactoryDefault();

    // Set neutral modes
    this.shooterTurretMotor.setNeutralMode(ShooterTurretConstants.kNeutralMode);

    // Set soft limits
    this.shooterTurretMotor.configForwardSoftLimitEnable(ShooterTurretConstants.kForwardSoftLimitEnabled);
    this.shooterTurretMotor.configReverseSoftLimitEnable(ShooterTurretConstants.kReverseSoftLimitEnabled);
    this.shooterTurretMotor.configForwardSoftLimitThreshold(ShooterTurretConstants.kForwardSoftLimitThreshold);
    this.shooterTurretMotor.configReverseSoftLimitThreshold(ShooterTurretConstants.kReverseSoftLimitThreshold);

    // Set feedback sensors
    this.shooterTurretMotor.configSelectedFeedbackSensor(ShooterTurretConstants.kFeedbackDevice);

    // Set sensor phases
    this.shooterTurretMotor.setSensorPhase(ShooterTurretConstants.kSensorPhase);

    // Reset encoder positions
    this.shooterTurretMotor.setSelectedSensorPosition(0);

    // Motion controllers
    this.feedbackController = new PIDController(
      ShooterTurretConstants.kP,
      ShooterTurretConstants.kI,
      ShooterTurretConstants.kD
    );

    // Offset
    this.offset = 0;

    // Telemtry
    SmartDashboard.putNumber("Shooter Turret Speed", 0);
    SmartDashboard.putBoolean("Shooter Turret At Forward Limit", false);
    SmartDashboard.putBoolean("Shooter Turret At Reverse Limit", false);
  }

  @Override
  public void periodic() {
    // Run the motor
    double output = this.feedbackController.calculate(this.offset, 0);
    this.shooterTurretMotor.set(ControlMode.PercentOutput, output);

    // Update telemtry
    SmartDashboard.putNumber("Shooter Turret Speed", this.shooterTurretMotor.get());
    SmartDashboard.putBoolean("Shooter Turret At Forward Limit", this.getAtForwardLimit());
    SmartDashboard.putBoolean("Shooter Turret At Reverse Limit", this.getAtReverseLimit());
  }

  /**
   * Set the offset seen by the limelight.
   *
   * The subsystem uses this to calculate how fast to run the motor.
   * @param offset The offset seen by the limelight.
   */
  public void setOffset(double offset) {
    this.offset = offset;
  }

  /**
   * Stop the shooter turret motor.
   */
  public void stop() {
    this.shooterTurretMotor.stopMotor();
  }

  public boolean atGoal() {
    return this.offset < ShooterTurretConstants.kGoalTolerance;
  }

  /**
   * Get if the motor is at the forward limit.
   * @return True if the motor is at the forward limit.
   */
  private boolean getAtForwardLimit() {
    double position = this.shooterTurretMotor.getSelectedSensorPosition();
    return position >= ShooterTurretConstants.kForwardSoftLimitThreshold;
  }

  /**
   * Get if the motor is at the reverse limit.
   * @return True if the motor is at the reverse limit.
   */
  private boolean getAtReverseLimit() {
    double position = this.shooterTurretMotor.getSelectedSensorPosition();
    return position <= ShooterTurretConstants.kReverseSoftLimitThreshold;
  }
}
