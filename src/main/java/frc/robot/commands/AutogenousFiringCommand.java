// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManagementConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ManagementSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class AutogenousFiringCommand extends CommandBase {
  // Subsystems
  private ShooterAngleSubsystem shooterAngleSubsystem;
  private ShooterFlywheelSubsystem shooterFlywheelSubsystem;
  private ShooterTurretSubsystem shooterTurretSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private ManagementSubsystem managementSubsystem;

  /** Creates a new FireShooterCommand. */
  public AutogenousFiringCommand(
    ShooterAngleSubsystem shooterAngleSubsystem,
    ShooterFlywheelSubsystem shooterFlywheelSubsystem,
    ShooterTurretSubsystem shooterTurretSubsystem,
    LimelightSubsystem limelightSubsystem,
    ManagementSubsystem managementSubsystem
  ) {
    this.shooterAngleSubsystem = shooterAngleSubsystem;
    this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
    this.shooterTurretSubsystem = shooterTurretSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.managementSubsystem = managementSubsystem;

    this.addRequirements(
      shooterAngleSubsystem,
      shooterFlywheelSubsystem,
      shooterTurretSubsystem,
      limelightSubsystem,
      managementSubsystem
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get setpoints
    double angleRequired = this.limelightSubsystem.getShooterAngleRequired();
    double velocityRequired = this.limelightSubsystem.getShooterVelocityRequired();
    double xOffset = this.limelightSubsystem.getXOffsetDegrees();

    // Run subsystems
    this.shooterAngleSubsystem.setGoal(angleRequired);
    this.shooterFlywheelSubsystem.setGoal(velocityRequired);
    this.shooterTurretSubsystem.setOffset(xOffset);

    // Check if all subsystems are ready
    if (!this.shooterAngleSubsystem.atGoal()) {
      // Stop firing balls
      this.managementSubsystem.stop();

      return;
    }
    if (!this.shooterFlywheelSubsystem.backAtGoal() || !this.shooterFlywheelSubsystem.frontAtGoal()) {
      // Stop firing balls
      this.managementSubsystem.stop();

      return;
    }
    if (!this.shooterTurretSubsystem.atGoal()) {
      // Stop firing balls
      this.managementSubsystem.stop();

      return;
    }

    // Fire balls
    this.managementSubsystem.set(ManagementConstants.kForwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterAngleSubsystem.stop();
    this.shooterFlywheelSubsystem.stop();
    this.shooterTurretSubsystem.stop();
    this.managementSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
