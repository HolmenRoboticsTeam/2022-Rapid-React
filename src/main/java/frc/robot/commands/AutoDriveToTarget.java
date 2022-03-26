// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveToTarget extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private PIDController pidController;

  private double targetDistance;
  private int atSetpointCounter;

  /** Creates a new AutoDriveForwardDistance. */
  public AutoDriveToTarget(DriveSubsystem driveSubsystem, double targetDistance) {
    this.driveSubsystem = driveSubsystem;
    this.targetDistance = targetDistance;
    this.pidController = new PIDController(1.0, 0.0, 0.0);
    this.atSetpointCounter = 0;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pidController.reset();
    this.pidController.setSetpoint(this.targetDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceCovered = this.driveSubsystem.getEncoderMeters();
    double speedY = this.pidController.calculate(distanceCovered);
    speedY = MathUtil.clamp(speedY, -DriveConstants.kAutoDriveForwardSpeed, DriveConstants.kAutoDriveForwardSpeed);

    this.driveSubsystem.arcadeDrive(0.0, speedY, 0.0);

    SmartDashboard.putNumber("Auto Drive Distance Covered", distanceCovered);
    SmartDashboard.putNumber("Auto Drive Distance Remaining", this.targetDistance - distanceCovered);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.arcadeDrive(0.0, 0.0, 0.0);
    SmartDashboard.putNumber("Auto Drive Speed", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceCovered = this.driveSubsystem.getEncoderMeters();
    if (Math.abs(distanceCovered - this.targetDistance) < 0.5) { // .5 to 5
      this.atSetpointCounter += 1;
    } else {
      this.atSetpointCounter = 0;
    }

    // Using PIDController.atSetpoint will return True even if we overshoot,
    // so use a counter to ensure we are at the setpoint for a period of time
    // before ending this command
    return this.atSetpointCounter >= 50;
  }
}
