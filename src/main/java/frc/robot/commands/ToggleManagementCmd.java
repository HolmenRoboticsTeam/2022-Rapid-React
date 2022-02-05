// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManagementSubsystem;

public class ToggleManagementCmd extends CommandBase {
  private final ManagementSubsystem managementSubsystem;
  private boolean open;

  /** Creates a new Toggelmanagment.
   * @param b*/
  public ToggleManagementCmd(ManagementSubsystem managementSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.open = false;
    this.managementSubsystem = managementSubsystem;
    addRequirements(managementSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.open = !this.open;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    managementSubsystem.toggelmagement(open);
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
