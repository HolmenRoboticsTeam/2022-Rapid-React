// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShooterCmd extends ParallelCommandGroup {   // button type parallels with the management system

  private ShooterFlywheelSubsystem shooterSubsystem;
  Joystick stick; // temporary

  public ShooterCmd(ShooterFlywheelSubsystem shooterSubsystem, Joystick stick) {

    this.shooterSubsystem = shooterSubsystem;
    this.stick = stick;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // System.out.println((this.stick.getRawAxis(3)));  // TODO: Remove
    shooterSubsystem.setMotor(this.stick.getRawAxis(3)); // Replace with ShooterConstants.kSpeed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
