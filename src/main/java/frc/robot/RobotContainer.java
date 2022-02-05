// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ManagementConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.ToggleIntakeCmd;
import frc.robot.commands.ToggleManagementCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManagementSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ManagementSubsystem managementSubsystem = new ManagementSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final Joystick joystick1 = new Joystick(OIConstants.kDriverJoystickPort);
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new ParallelCommandGroup(new ShooterCmd(shooterSubsystem, joystick1.getRawButtonPressed(OIConstants.kShooterButtonIdx)),
    new ToggleManagementCmd(managementSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick1, OIConstants.kIntakeButtonIdx)
            .whenPressed(new ToggleIntakeCmd(intakeSubsystem));
    new JoystickButton(joystick1, OIConstants.kManagementButtonIdx)
            .whenPressed(new ToggleManagementCmd(managementSubsystem));
    new JoystickButton(joystick1, OIConstants.kManagementButtonIdx)
            .whenPressed(new ToggleManagementCmd(managementSubsystem));

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
