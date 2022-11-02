// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleConstants;

public class ShooterAngleSubsystem extends SubsystemBase {
  // Motors
  private WPI_TalonSRX shooterAngleMotor;

  // Goal and tolerance
  private double goal;
  private double tolerance;

  /** Creates a new ShooterAngleSubsystem. */
  public ShooterAngleSubsystem() {
    // Motors
    this.shooterAngleMotor = new WPI_TalonSRX(ShooterAngleConstants.kShooterAngleMotorPort);

    // Reset motor configs
    this.shooterAngleMotor.configFactoryDefault();

    // Set neutral mode
    this.shooterAngleMotor.setNeutralMode(ShooterAngleConstants.kNeutralMode);

    // Set feedback sensor
    this.shooterAngleMotor.configSelectedFeedbackSensor(ShooterAngleConstants.kFeedbackDevice);

    // Set sensor phase
    this.shooterAngleMotor.setSensorPhase(ShooterAngleConstants.kSensorPhase);

    // Reset encoder position
    this.shooterAngleMotor.setSelectedSensorPosition(0);

    // Set built-in motion profiles
    this.shooterAngleMotor.config_kP(0, ShooterAngleConstants.kP);
    this.shooterAngleMotor.config_kI(0, ShooterAngleConstants.kI);
    this.shooterAngleMotor.config_kD(0, ShooterAngleConstants.kD);
    this.shooterAngleMotor.config_kF(0, ShooterAngleConstants.kF);

    // Goal and tolerance
    this.goal = 0;
    this.tolerance = ShooterAngleConstants.kGoalTolerance;

    // Telemetry
    SmartDashboard.putNumber("Shooter Angle Speed", 0);
    SmartDashboard.putNumber("Shooter Angle", 0);
    SmartDashboard.putBoolean("Shooter Angle Ready", false);
  }

  @Override
  public void periodic() {
    // Run the motor continuously
    double goalNative = this.goal / ShooterAngleConstants.kDegreesPerPulse;
    this.shooterAngleMotor.set(TalonSRXControlMode.Position, goalNative);

    // Update telemetry
    SmartDashboard.putNumber("Shooter Angle Speed", this.shooterAngleMotor.get());
    SmartDashboard.putNumber("Shooter Angle", this.getCurrentAngle());
    SmartDashboard.putBoolean("Shooter Angle Ready", this.atGoal());
  }

  /**
   * Set the desired goal (angle) for the shooter.
   * @param goal desired angle in degrees.
   */
  public void setGoal(double goal) {
    this.goal = 90.0 - goal;  // Subtract goal from 90 to base off x-axis, not z
  }

  /**
   * Stop the shooter angle motor;
   */
  public void stop() {
    this.shooterAngleMotor.stopMotor();
  }

  /**
   * Get the current angle of the shooter.
   * @return The angle of the shooter in degrees.
   */
  public double getCurrentAngle() {
    return this.shooterAngleMotor.getSelectedSensorPosition() * ShooterAngleConstants.kDegreesPerPulse;
  }

  /**
   * Check if the shooter is at the desired angle.
   * @return True if the shooter is at the current angle.
   */
  public boolean atGoal() {
    double currentAngle = this.getCurrentAngle();
    double deviation = Math.abs(currentAngle - this.goal) / this.goal;
    return deviation <= this.tolerance;
  }
}
