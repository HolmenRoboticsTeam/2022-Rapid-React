// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManagementConstants;

public class ManagementSubsystem extends SubsystemBase {
  // Motors
  private WPI_VictorSPX managementMotor;

  /** Creates a new ManagementSubsystem. */
  public ManagementSubsystem() {
    // Motors
    this.managementMotor = new WPI_VictorSPX(ManagementConstants.kManagementMotorPort);

    // Reset motor configs
    this.managementMotor.configFactoryDefault();

    // Telemetry
    SmartDashboard.putNumber("Management Motor Speed", 0);
  }

  @Override
  public void periodic() {
    // Update telemtry
    SmartDashboard.putNumber("Management Motor Speed", this.managementMotor.get());
  }

  /**
   * Run the motor at a specific speed.
   * @param speed the speed to run the motor at.
   */
  public void set(double speed) {
    this.managementMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Stop the management motor.
   */
  public void stop() {
    this.managementMotor.stopMotor();
  }
}
