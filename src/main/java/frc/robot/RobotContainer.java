// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.RotateShooterCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.ToggleIntakeCmd;
import frc.robot.commands.ToggleManagementCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ManagementSubsystem;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ManagementSubsystem managementSubsystem = new ManagementSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ShooterRotationSubsystem shooterRotationSubsystem = new ShooterRotationSubsystem();
  private final LimeLightSubsystem limelightSubsystem = new LimeLightSubsystem();

  private final Joystick joystick1 = new Joystick(OIConstants.kDriverJoystickPort); //left joystick

  private final Joystick joystick2 = new Joystick(OIConstants.kDriverJoystickPort2); // right joystick

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new ParallelCommandGroup(new ShooterCmd(shooterSubsystem, joystick2.getRawButtonPressed(OIConstants.kShooterButtonIdx)),
    new ToggleManagementCmd(managementSubsystem));

   // Configure the button bindings
      configureButtonBindings();

      driveSubsystem.setDefaultCommand(
        new DriveCmd(driveSubsystem, () -> -this.joystick1.getY(), () -> this.joystick1.getX(), () -> this.joystick2.getX()));

      shooterRotationSubsystem.setDefaultCommand(new RotateShooterCmd(shooterRotationSubsystem, limelightSubsystem, true)); // set to constantly track reflective tape
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick2, OIConstants.kIntakeButtonIdx)
            .whenPressed(new ToggleIntakeCmd(intakeSubsystem));

    new JoystickButton(joystick2, OIConstants.kManagementButtonIdx)
            .whenPressed(new ToggleManagementCmd(managementSubsystem));

    new JoystickButton(joystick1, OIConstants.kClimberButtonUpIdx)
            .whenPressed(new ClimberCmd(climberSubsystem, 1));
    new JoystickButton(joystick1, OIConstants.kClimberButtonDownIdx)
            .whileActiveOnce(new ClimberCmd(climberSubsystem, -1));

    new JoystickButton(joystick1, OIConstants.kRotationButtonIdx)
            .whenPressed(new RotateShooterCmd(shooterRotationSubsystem, limelightSubsystem, false));
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }*/
}
/*
        elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(elevatorSubsystem, 0));
        intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));
*/