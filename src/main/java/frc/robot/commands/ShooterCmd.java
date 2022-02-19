// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends ParallelCommandGroup {   // button type parallels with the management system

  private final ShooterSubsystem ShooterSubsystem;
  private final boolean on;

  public ShooterCmd(ShooterSubsystem shooterSubsystem, boolean on) {

    this.ShooterSubsystem = shooterSubsystem;
    this.on = on;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {} //

  @Override
  public void execute() {
    ShooterSubsystem.ShooterOn(on);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
