package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

    private WPI_VictorSPX leftClimberMotor = new WPI_VictorSPX(ClimberConstants.kLeftClimberMotorPort);
    private WPI_VictorSPX rightClimberMotor = new WPI_VictorSPX(ClimberConstants.kRightClimberMotorPort);

    public ClimberSubsystem() {}

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Left Climber Speed", leftClimberMotor.get());
      SmartDashboard.putNumber("Right Climber Speed", rightClimberMotor.get());
    }

    public void setMotor(double speed) {
      leftClimberMotor.set(speed);
      rightClimberMotor.set(speed);
  }
}
