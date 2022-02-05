package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
  private WPI_TalonSRX shootorMotor = new WPI_TalonSRX(ShooterConstants.kShootertMotorPort);

    public ShooterSubsystem() {

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void ShooterOn(boolean on) {
      if (on) {
        shootorMotor.set(ShooterConstants.shooterRunSpeed);
      } else {
        shootorMotor.set(ShooterConstants.shooterStopSpeed);
      }
    }
}
