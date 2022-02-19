// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
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
    this.limelightOn = limelightOn;

    SmartDashboard.putNumber("Shooter Minimum Speed", ShooterConstants.kMinimumRotationSpeed);
    SmartDashboard.putNumber("Shooter Maximum Speed", ShooterConstants.kMaximumRotationSpeed);
    SmartDashboard.putNumber("Shooter Rotation Slow Down Angle", ShooterConstants.rotationConstantAngle);

    addRequirements(shooterRotationSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.limelightOn = !this.limelightOn;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tX = this.limeLight.getDoubleTX();

    double minimumSpeed = SmartDashboard.getNumber("Shooter Minimum Speed", ShooterConstants.kMinimumRotationSpeed);
    double maximumSpeed = SmartDashboard.getNumber("Shooter Maximum Speed", ShooterConstants.kMaximumRotationSpeed);
    double rotationSlowDownAngle = SmartDashboard.getNumber("Shooter Rotation Slow Down Angle", ShooterConstants.rotationConstantAngle);
    double speed = maximumSpeed;
    speed = tX > 0 ? maximumSpeed : -maximumSpeed;

    if(Math.abs(tX) < rotationSlowDownAngle) {
      speed = minimumSpeed;
    }

    System.out.println(speed);
    shooterRotationSubsystem.RotationToggle(limelightOn);

    if(limeLight.getDoubleTX() != 0.0){
      shooterRotationSubsystem.ShooterRotation(speed);

      }

      double tY = this.limeLight.getDoubleTY();
      double angleToGoalDegrees = (ShooterConstants.kLimeLightAngle + tY);
      double angleToGoalRadians = (angleToGoalDegrees * (Math.PI/180.0));

      double distanceFromLimelightToGoalInches = ((ShooterConstants.kHeightOfTarget - ShooterConstants.kLimeLightHeightFromGround)/(Math.tan(angleToGoalRadians)));
      System.out.println(distanceFromLimelightToGoalInches);

      // double tV = this.limeLight.getDoubleTV();
      // double kP = ShooterConstants.kPControlConstant;
      // double steering_adjust = kP * tX;

      // if ( tV == 0.0) {
      //   steering_adjust = 0.3f;
      // }
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
