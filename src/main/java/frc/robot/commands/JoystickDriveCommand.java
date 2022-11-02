// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDriveCommand extends CommandBase {
  // Subsystems
  private DriveSubsystem driveSubsystem;

  // Joysticks
  private Joystick leftJoystick;

  /** Creates a new JoystickDriveCommand. */
  public JoystickDriveCommand(DriveSubsystem driveSubsystem, Joystick leftJoystick) {
    this.driveSubsystem = driveSubsystem;
    this.leftJoystick = leftJoystick;

    this.addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = -this.leftJoystick.getY();  // Negate because joystick up is more negative
    double turn = this.leftJoystick.getX();

    // Drive the robot
    this.driveSubsystem.arcadeDrive(throttle, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
