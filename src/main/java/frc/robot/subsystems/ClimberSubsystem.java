package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

    private WPI_VictorSPX climberMotor = new WPI_VictorSPX(ClimberConstants.kMainMotorPort);

    public ClimberSubsystem() {
    }

    @Override
    public void periodic() {} //

    public void setDirection(int direction) {
      if (direction > 0)
        climberMotor.set(ClimberConstants.kSpeed);
      else if (direction < 0)
        climberMotor.set(-ClimberConstants.kSpeed);
      else
        climberMotor.set(0);
      
  }

}
