// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ManualShooterAngle extends CommandBase {
  private final ShooterAngleSubsystem shooterAngleSubsystem;
  private final Joystick leftJoystick;
  /** Creates a new ManualShooterAngle. */
  public ManualShooterAngle(ShooterAngleSubsystem shooterAngleSubsystem, Joystick leftJoystick) {
    this.shooterAngleSubsystem = shooterAngleSubsystem;
    this.leftJoystick = leftJoystick;
    addRequirements(shooterAngleSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooterAngleSubsystem.setMotor(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterAngleSubsystem.setMotor(this.leftJoystick.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended");
    this.shooterAngleSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
