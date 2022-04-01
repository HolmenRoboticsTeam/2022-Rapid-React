// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ManagementConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AngleShooterCmd;
import frc.robot.commands.AutoDriveToTarget;
import frc.robot.commands.RunClimberCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.RotateShooterCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.RunIntakeCmd;
import frc.robot.commands.RunManagementCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ManagementSubsystem;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class RobotContainer {

  private final UsbCamera camera1 = CameraServer.startAutomaticCapture();

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ManagementSubsystem managementSubsystem = new ManagementSubsystem();
  private final ShooterFlywheelSubsystem shooterSubsystem = new ShooterFlywheelSubsystem();
  private final ShooterAngleSubsystem shooterVerticalSubsystem = new ShooterAngleSubsystem();
  private final ShooterRotationSubsystem shooterRotationSubsystem = new ShooterRotationSubsystem();
  private final LimeLightSubsystem limelightSubsystem = new LimeLightSubsystem();

  private final Joystick leftHandedJoystick = new Joystick(OIConstants.kLeftHandedJoystickPort);
  private final Joystick rightHandedJoystick = new Joystick(OIConstants.kRightHandedJoystickPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(
      new DriveCmd(
        driveSubsystem,
        () -> this.rightHandedJoystick.getY(),  // Forward-Back
        () -> this.leftHandedJoystick.getX() * 0.0,  // Straffe Left-Right (not needed anymore)
        () -> this.rightHandedJoystick.getX() * 0.80  // Rotate
      )
    );

    shooterRotationSubsystem.setDefaultCommand(
      new RotateShooterCmd(shooterRotationSubsystem, limelightSubsystem)
    );

    shooterVerticalSubsystem.setDefaultCommand(
      new AngleShooterCmd(shooterVerticalSubsystem, () -> limelightSubsystem.getDoubleTA(), leftHandedJoystick)
    );
    // climberSubsystem.setDefaultCommand(new RunClimberCmd(climberSubsystem, ClimberConstants.kExtendSpeed, leftHandedJoystick));

    camera1.setResolution(160, 120);
    camera1.setFPS(30);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Run intake when button is held
    new JoystickButton(rightHandedJoystick, OIConstants.kIntakeButtonIdx)
      .whenHeld(new RunIntakeCmd(intakeSubsystem));

    // Run management forward when button is held
    new JoystickButton(rightHandedJoystick, OIConstants.kManagementButtonIdx)
      .whenHeld(new RunManagementCmd(managementSubsystem, ManagementConstants.kSpeed));

    // Run management backwards when button is held
    new JoystickButton(rightHandedJoystick, OIConstants.kManagementButton2)
      .whenHeld(new RunManagementCmd(managementSubsystem, ManagementConstants.kSpeedAlt));

    // Extend climber when button is held
    new JoystickButton(leftHandedJoystick, OIConstants.kClimberButtonUpIdx)
      .whenHeld(new RunClimberCmd(climberSubsystem, ClimberConstants.kExtendSpeed, leftHandedJoystick));

    // // Retract climber when button is held
    new JoystickButton(leftHandedJoystick, OIConstants.kClimberButtonDownIdx)
      .whenHeld(new RunClimberCmd(climberSubsystem,  ClimberConstants.kRetractSpeed, leftHandedJoystick));

    new JoystickButton(rightHandedJoystick, OIConstants.kShooterButtonIdx)
        .whenHeld(new ShooterCmd(shooterSubsystem, ShooterConstants.kShooterRunSpeed, rightHandedJoystick));

    // Run intake and management simultaneously
    new JoystickButton(rightHandedJoystick, OIConstants.kManagementAndIntakeIdx)
        .whenHeld(new RunIntakeCmd(intakeSubsystem))
        .whenHeld(new RunManagementCmd(managementSubsystem, ManagementConstants.kSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
      new AutoDriveToTarget(this.driveSubsystem, 2.0),
      new WaitCommand(1),
      new ParallelCommandGroup(
        new ShooterCmd(shooterSubsystem, ShooterConstants.kShooterAutoRunSpeed, rightHandedJoystick),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new ParallelCommandGroup (
            new RunIntakeCmd(intakeSubsystem),
            new RunManagementCmd(managementSubsystem, ManagementConstants.kSpeed))
        )
      )
    );
  }
}
/*
 * elevatorSubsystem.setDefaultCommand(new
 * ElevatorJoystickCmd(elevatorSubsystem, 0));
 * intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));
 */