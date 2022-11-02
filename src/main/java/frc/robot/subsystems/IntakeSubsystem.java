// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Motors
  private WPI_VictorSPX intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Motors
    this.intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorPort);

    // Reset motor configs
    this.intakeMotor.configFactoryDefault();

    // Telemetry
    SmartDashboard.putNumber("Intake Speed", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Speed", this.intakeMotor.get());
  }

  /**
   * Run the motor at a specific speed.
   * @param speed how fast to run the motor [-1, 1]
   */
  public void set(double speed) {
    this.intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stop the intake motor.
   */
  public void stop() {
    this.intakeMotor.stopMotor();
  }
}
