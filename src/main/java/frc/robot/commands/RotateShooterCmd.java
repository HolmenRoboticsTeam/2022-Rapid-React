// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RotateShooterCmd extends CommandBase {
  /** Creates a new RotateShooterCmd. */
  private final ShooterSubsystem ShooterSubsystem;
  private final LimeLightSubsystem limeLight;

  public RotateShooterCmd(ShooterSubsystem shooterSubsystem, LimeLightSubsystem limeLight) {

    this.ShooterSubsystem = shooterSubsystem;
    this.limeLight = limeLight;

    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limeLight.getDoubleTX() != 0.0){
      ShooterSubsystem.ShooterRotation(limeLight.getDoubleTX() / 5.0);

    }
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
