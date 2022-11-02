// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputConstants;
import frc.robot.commands.AngleShooterCommand;
import frc.robot.commands.AutogenousFiringCommand;
import frc.robot.commands.ExtendClimbersCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.RetractClimbersCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.RunIntakeForwardCommand;
import frc.robot.commands.RunIntakeManagementForwardParallelCommand;
import frc.robot.commands.RunManagementForwardCommand;
import frc.robot.commands.RunManagementReverseCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ManagementSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ClimberSubsystem climberSubsystem;
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private ManagementSubsystem managementSubsystem;
  private ShooterAngleSubsystem shooterAngleSubsystem;
  private ShooterFlywheelSubsystem shooterFlywheelSubsystem;
  private ShooterTurretSubsystem shooterTurretSubsystem;

  private ExtendClimbersCommand extendClimbersCommand;
  private RetractClimbersCommand retractClimbersCommand;
  private JoystickDriveCommand joystickDriveCommand;
  private RunIntakeForwardCommand runIntakeForwardCommand;
  private RunManagementForwardCommand runManagementForwardCommand;
  private RunManagementReverseCommand runManagementReverseCommand;
  private AngleShooterCommand angleShooterCommand;
  private RotateShooterCommand rotateShooterCommand;

  private RunIntakeManagementForwardParallelCommand runIntakeManagementForwardParallelCommand;

  private AutogenousFiringCommand autogenousFiringCommand;

  // The input joysticks and buttons are defined here...
  private Joystick leftJoystick;
  private Joystick rightJoystick;

  private JoystickButton climberExtendButton;
  private JoystickButton climberRetractButton;

  private JoystickButton runIntakeForwardButton;

  private JoystickButton runManagementForwardButton;
  private JoystickButton runManagementReverseButton;

  private JoystickButton runIntakeManagementForwardButton;

  private JoystickButton autogenousFiringButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Create the subsystems and commands
    this.createSubsystemsCommands();

    // Create the joysticks and buttons
    this.createJoysticksButtons();

    // Configure the button bindings
    this.configureButtonBindings();

    // Set default commands for subsystems
    this.setDefaultCommands();

    // Start the camera
    CameraServer.startAutomaticCapture();
  }

  /**
   * Create robot subsystems and commands in this method.
   */
  private void createSubsystemsCommands() {
    this.climberSubsystem = new ClimberSubsystem();
    this.driveSubsystem = new DriveSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    this.limelightSubsystem = new LimelightSubsystem();
    this.managementSubsystem = new ManagementSubsystem();
    this.shooterAngleSubsystem = new ShooterAngleSubsystem();
    this.shooterFlywheelSubsystem = new ShooterFlywheelSubsystem();
    this.shooterTurretSubsystem = new ShooterTurretSubsystem();

    this.extendClimbersCommand = new ExtendClimbersCommand(this.climberSubsystem);
    this.retractClimbersCommand = new RetractClimbersCommand(this.climberSubsystem);
    this.joystickDriveCommand = new JoystickDriveCommand(this.driveSubsystem, this.leftJoystick);
    this.runIntakeForwardCommand = new RunIntakeForwardCommand(this.intakeSubsystem);
    this.runManagementForwardCommand = new RunManagementForwardCommand(this.managementSubsystem);
    this.runManagementReverseCommand = new RunManagementReverseCommand(this.managementSubsystem);
    this.angleShooterCommand = new AngleShooterCommand(this.shooterAngleSubsystem, this.limelightSubsystem);
    this.rotateShooterCommand = new RotateShooterCommand(this.shooterTurretSubsystem, this.limelightSubsystem);
    this.runIntakeManagementForwardParallelCommand = new RunIntakeManagementForwardParallelCommand(
      this.runIntakeForwardCommand,
      this.runManagementForwardCommand
    );
    this.autogenousFiringCommand = new AutogenousFiringCommand(
      this.shooterAngleSubsystem,
      this.shooterFlywheelSubsystem,
      this.shooterTurretSubsystem,
      this.limelightSubsystem,
      this.managementSubsystem
    );
  }

  /**
   * Create joysticks and joystick buttons in this method.
   */
  private void createJoysticksButtons() {
    // Joysticks
    this.leftJoystick = new Joystick(InputConstants.kLeftJoystickPort);
    this.rightJoystick = new Joystick(InputConstants.kRightJoystickPort);

    // Joystick buttons
    this.climberExtendButton = new JoystickButton(this.leftJoystick, InputConstants.kClimberExtendLeftJButton);
    this.climberRetractButton = new JoystickButton(this.leftJoystick, InputConstants.kClimberRetractLeftJButton);
    this.runIntakeForwardButton = new JoystickButton(this.rightJoystick, InputConstants.kIntakeForwardRightJButton);
    this.runManagementForwardButton = new JoystickButton(this.rightJoystick, InputConstants.kManagementForwardRightJButton);
    this.runManagementReverseButton = new JoystickButton(this.rightJoystick, InputConstants.kManagementReverseRightJButton);
    this.runIntakeManagementForwardButton = new JoystickButton(this.rightJoystick, InputConstants.kIntakeAndManagementForwardRightJButton);
    this.autogenousFiringButton = new JoystickButton(this.rightJoystick, InputConstants.kAutogenousFiringRightJButton);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Climber
    this.climberExtendButton.whenHeld(this.extendClimbersCommand);
    this.climberRetractButton.whenHeld(this.retractClimbersCommand);

    // Intake
    this.runIntakeForwardButton.whenHeld(this.runIntakeForwardCommand);

    // Management
    this.runManagementForwardButton.whenHeld(this.runManagementForwardCommand);
    this.runManagementReverseButton.whenHeld(this.runManagementReverseCommand);

    // Management and Intake
    this.runIntakeManagementForwardButton.whenHeld(this.runIntakeManagementForwardParallelCommand);

    // Shooter
    this.autogenousFiringButton.whenHeld(this.autogenousFiringCommand);
  }

  /**
   * Set default commands for subsystems in this method.
   */
  private void setDefaultCommands() {
    // Drivebase
    this.driveSubsystem.setDefaultCommand(this.joystickDriveCommand);

    // Shooter
    this.shooterAngleSubsystem.setDefaultCommand(this.angleShooterCommand);
    this.shooterTurretSubsystem.setDefaultCommand(this.rotateShooterCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
