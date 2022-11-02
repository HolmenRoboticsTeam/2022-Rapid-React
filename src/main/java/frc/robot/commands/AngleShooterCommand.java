// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class AngleShooterCommand extends CommandBase {
  private ShooterAngleSubsystem shooterAngleSubsystem;
  private LimelightSubsystem limelightSubsystem;

  /** Creates a new AngleShooterCommand. */
  public AngleShooterCommand(ShooterAngleSubsystem shooterAngleSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterAngleSubsystem = shooterAngleSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    // We don't require the limelightSubsystem
    // Since we are not requesting the same information from it across multiple subsystems
    // simultaneously, we do not want to interrupt other subsytems while reading from it
    // Now, if this is not the case, then we would need to look into requiring it...
    this.addRequirements(shooterAngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double requiredAngle = this.limelightSubsystem.getShooterAngleRequired();
    this.shooterAngleSubsystem.setGoal(Units.radiansToDegrees(requiredAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterAngleSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
