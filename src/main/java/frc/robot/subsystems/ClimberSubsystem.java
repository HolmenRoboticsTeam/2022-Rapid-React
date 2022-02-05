package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
 // private int kMotor = new int(ElevatorConstants.kMotorPort); // Int is temporary....

 //motortype sure for now!
    private WPI_VictorSPX climberMotor = new WPI_VictorSPX(ClimberConstants.kMainMotorPort);

    public ClimberSubsystem() {
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    @Override
    public void setPosition(boolean open) {
      if (open) {
        climberMotor.set(ClimberConstants.kSpeed);
      } else {
        climberMotor.set(0);
      }
  }

}
