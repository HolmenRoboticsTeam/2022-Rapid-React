package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterFlywheelSubsystem extends SubsystemBase{
  private WPI_TalonSRX shooterMotorFront = new WPI_TalonSRX(ShooterConstants.kShooterFlywheelFrontMotorPort);
  private WPI_TalonSRX shooterMotorBack = new WPI_TalonSRX(ShooterConstants.kShooterFlywheelBackMotorPort);

  public ShooterFlywheelSubsystem() {
    shooterMotorFront.configFactoryDefault();
    shooterMotorBack.configFactoryDefault();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Front Flywheel Speed", shooterMotorFront.get());
    SmartDashboard.putNumber("Shooter Back Flywheel Speed", shooterMotorBack.get());
  }

  public void setMotor(double speed){
    shooterMotorFront.set(speed);
    shooterMotorBack.set(speed);
  }
}
