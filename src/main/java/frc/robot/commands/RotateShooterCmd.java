// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterRotationSubsystem;

public class RotateShooterCmd extends CommandBase {
  /** Creates a new RotateShooterCmd. */
  private final ShooterRotationSubsystem shooterRotationSubsystem;
  private final LimeLightSubsystem limeLight;
  private boolean limelightOn; // For if we want to assign a button to turn on the rotating part

  public RotateShooterCmd(ShooterRotationSubsystem shooterRotationSubsystem, LimeLightSubsystem limeLight, boolean limelightOn) {

    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.limeLight = limeLight;
    this.limelightOn = false;

    addRequirements(shooterRotationSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.limelightOn = !this.limelightOn;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterRotationSubsystem.RotationToggle(limelightOn);
  
    if(limeLight.getDoubleTX() != 0.0){
      shooterRotationSubsystem.ShooterRotation(limeLight.getDoubleTX() / 5.0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
