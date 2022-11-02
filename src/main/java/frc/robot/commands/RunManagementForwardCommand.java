// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManagementConstants;
import frc.robot.subsystems.ManagementSubsystem;

public class RunManagementForwardCommand extends CommandBase {
  // Subsystems
  private ManagementSubsystem managementSubsystem;

  /** Creates a new RunManagementForwardCommand. */
  public RunManagementForwardCommand(ManagementSubsystem managementSubsystem) {
    this.managementSubsystem = managementSubsystem;

    this.addRequirements(managementSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.managementSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.managementSubsystem.set(ManagementConstants.kForwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.managementSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
