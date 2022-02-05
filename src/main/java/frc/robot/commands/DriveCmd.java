// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  // joystick commands are here
 // private final DifferentialDrive differentialDrive;
  private final Joystick joystick;
 // private final int yAxis;
 // private final int xAxis;

  /** Creates a new DriveCmd. */
  public DriveCmd(DriveSubsystem driveSubsystem, Joystick joystick) {
    //this. initilizes it
    this.driveSubsystem = driveSubsystem;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
    return false;
  }
}