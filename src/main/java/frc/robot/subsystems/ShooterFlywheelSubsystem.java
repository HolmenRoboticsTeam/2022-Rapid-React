// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterFlywheelConstants;

public class ShooterFlywheelSubsystem extends SubsystemBase {
  // Motors
  private WPI_TalonFX shooterMotorFront;
  private WPI_TalonFX shooterMotorBack;

  // Goal and tolerance
  private double goal;
  private double tolerance;

  /** Creates a new ShooterFlywheelSubsystem. */
  public ShooterFlywheelSubsystem() {
    // Motors
    this.shooterMotorFront = new WPI_TalonFX(ShooterFlywheelConstants.kFrontMotorPort);
    this.shooterMotorBack = new WPI_TalonFX(ShooterFlywheelConstants.kBackMotorPort);

    // Reset motor configs
    this.shooterMotorFront.configFactoryDefault();
    this.shooterMotorBack.configFactoryDefault();

    // Set neutral modes
    this.shooterMotorFront.setNeutralMode(ShooterFlywheelConstants.kFrontNeutralMode);
    this.shooterMotorBack.setNeutralMode(ShooterFlywheelConstants.kBackNeutralMode);

    // Reset encoder positions
    this.shooterMotorFront.setSelectedSensorPosition(0);
    this.shooterMotorBack.setSelectedSensorPosition(0);

    // Set built-in motion profiles
    this.shooterMotorFront.config_kP(0, ShooterFlywheelConstants.kFrontKP);
    this.shooterMotorFront.config_kI(0, ShooterFlywheelConstants.kFrontKI);
    this.shooterMotorFront.config_kD(0, ShooterFlywheelConstants.kFrontKD);
    this.shooterMotorFront.config_kF(0, ShooterFlywheelConstants.kFrontKF);

    this.shooterMotorBack.config_kP(0, ShooterFlywheelConstants.kBackKP);
    this.shooterMotorBack.config_kI(0, ShooterFlywheelConstants.kBackKI);
    this.shooterMotorBack.config_kD(0, ShooterFlywheelConstants.kBackKD);
    this.shooterMotorBack.config_kF(0, ShooterFlywheelConstants.kBackKF);

    // Goal and tolerance
    this.goal = 0;
    this.tolerance = ShooterFlywheelConstants.kGoalTolerance;

    // Telemetry
    SmartDashboard.putBoolean("Shooter Flywheel Front Ready", false);
    SmartDashboard.putBoolean("Shooter Flywheel Back Ready", false);
    SmartDashboard.putNumber("Shooter Flywheel Front Speed mps", 0);
    SmartDashboard.putNumber("Shooter Flywheel Back Speed mps", 0);
    SmartDashboard.putNumber("Shooter Flywheel Goal mps", 0);
  }

  @Override
  public void periodic() {
    // Run the motors
    double goal100ms = this.velocityToNativeUnits(this.goal);
    this.shooterMotorFront.set(TalonFXControlMode.Velocity, goal100ms);
    this.shooterMotorBack.set(TalonFXControlMode.Velocity, goal100ms);

    // Update telemetry
    SmartDashboard.putBoolean("Shooter Flywheel Front Ready", this.frontAtGoal());
    SmartDashboard.putBoolean("Shooter Flywheel Back Ready", this.backAtGoal());
    SmartDashboard.putNumber("Shooter Flywheel Front Speed mps", this.getFrontVelocity());
    SmartDashboard.putNumber("Shooter Flywheel Back Speed mps", this.getBackVelocity());
    SmartDashboard.putNumber("Shooter Flywheel Goal mps", this.goal);
  }

  /**
   * Get the velocity of the front flywheel in m/s.
   * @return The velocity of front wheel in m/s.
   */
  public double getFrontVelocity() {
    double sensorCountsPer100ms = this.shooterMotorFront.getSelectedSensorVelocity();
    return this.nativeUnitsToVelocity(sensorCountsPer100ms);
  }

  /**
   * Get the velocity of the back flywheel in m/s.
   * @return The velocity of back wheel in m/s.
   */
  public double getBackVelocity() {
    double sensorCountsPer100ms = this.shooterMotorBack.getSelectedSensorVelocity();
    return this.nativeUnitsToVelocity(sensorCountsPer100ms);
  }

  public boolean frontAtGoal() {
    double currentVelocity = this.getFrontVelocity();
    double percentDifference = Math.abs(this.goal - currentVelocity) / ((this.goal + currentVelocity) / 2.0);
    return percentDifference <= this.tolerance;
  }

  public boolean backAtGoal() {
    double currentVelocity = this.getBackVelocity();
    double percentDifference = Math.abs(this.goal - currentVelocity) / ((this.goal + currentVelocity) / 2.0);
    return percentDifference <= this.tolerance;
  }

  /**
   * Set the desired velocity of the shooter in m/s.
   * @param goal the desired velocity in m/s.
   */
  public void setGoal(double goal) {
    this.goal = goal;
  }

  public void stop() {
    this.shooterMotorFront.stopMotor();
    this.shooterMotorBack.stopMotor();
  }

  /**
   * Calculate the velocity of a wheel in m/s.
   * @param sensorCountsPer100ms the velocity of the encoder in native units/100ms.
   * @return The velocity of a wheel in m/s.
   */
  private double nativeUnitsToVelocity(double sensorCountsPer100ms) {
    double motorRotationsPer100ms = sensorCountsPer100ms / ShooterFlywheelConstants.kTicksPerRev;
    double motorRotationsPerSecond = motorRotationsPer100ms * ShooterFlywheelConstants.k100msPerSecond;
    double wheelRotationsPerSecond = motorRotationsPerSecond / ShooterFlywheelConstants.kGearRatio;
    double velocityMetersPerSecond = wheelRotationsPerSecond * ShooterFlywheelConstants.kWheelCircumferenceMeters;
    return velocityMetersPerSecond;
  }

  /**
   * Calculate the velocity of a wheel in native units/100ms.
   * @param velocityMetersPerSecond the velocity of the flywheel in m/s.
   * @return The velocity of the encoder in native units/100ms.
   */
  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond / (2 * Math.PI * ShooterFlywheelConstants.kWheelRadiusMeters);
    double motorRotationsPerSecond = wheelRotationsPerSecond * ShooterFlywheelConstants.kGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / ShooterFlywheelConstants.k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * ShooterFlywheelConstants.kTicksPerRev);
    return sensorCountsPer100ms;
  }
}
