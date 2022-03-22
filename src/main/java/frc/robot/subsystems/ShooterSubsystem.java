package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
  // private double gearBoxRatio = ShooterConstants.kConstantGearRatio;
  // private double gearDiameter = ShooterConstants.kGearDiameter;

  private WPI_TalonSRX shooterMotorFront = new WPI_TalonSRX(ShooterConstants.kShootertMotorPortFront);
  private WPI_TalonSRX shooterMotorBack = new WPI_TalonSRX(ShooterConstants.kShooterMotorPortBack);

    public ShooterSubsystem() {
      shooterMotorFront.configFactoryDefault();
      shooterMotorBack.configFactoryDefault();
    }

    //public double getSpeedMeters() {
     // return (shootorMotor.getSelectedSensorVelocity() / 4096.0 * gearBoxRatio * (gearDiameter * Math.PI));
     // (kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
    // }
    @Override
    public void periodic() {} //

    public void setMotor(double speed){
      shooterMotorFront.set(speed);
      shooterMotorBack.set(-speed);
    }

}
