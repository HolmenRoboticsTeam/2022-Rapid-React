// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ZeroShooterAngle extends CommandBase {
  private final ShooterAngleSubsystem shooterAngleSubsystem;
  private final Joystick leftHandedJoystick;
  /** Creates a new ZeroShooterAngle. */
  public ZeroShooterAngle(ShooterAngleSubsystem shooterAngleSubsystem, Joystick leftHandedJoystick) {
    this.shooterAngleSubsystem = shooterAngleSubsystem;
    this.leftHandedJoystick = leftHandedJoystick;
    addRequirements(shooterAngleSubsystem);
  }

  // assign to a button, create a method called "reset"

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooterAngleSubsystem.setMotor(0);
    this.shooterAngleSubsystem.reset();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
