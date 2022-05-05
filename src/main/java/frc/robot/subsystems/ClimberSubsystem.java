package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

    private WPI_VictorSPX leftClimberMotor = new WPI_VictorSPX(ClimberConstants.kLeftClimberMotorPort);
    private WPI_VictorSPX rightClimberMotor = new WPI_VictorSPX(ClimberConstants.kRightClimberMotorPort);

    private DigitalInput leftLimit = new DigitalInput(ClimberConstants.kLeftDigitalInput);
    private DigitalInput rightLimit = new DigitalInput(ClimberConstants.kRightDigitalInput);

    public ClimberSubsystem() {
      leftClimberMotor.setNeutralMode(NeutralMode.Brake);
      rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Left Climber Speed", leftClimberMotor.get());
      SmartDashboard.putNumber("Right Climber Speed", rightClimberMotor.get());
    }

    public boolean isLeftClimberAtLimit() {
      return leftLimit.get() == false;
    }

    public boolean isRightClimberAtLimit() {
      return rightLimit.get() == false;
    }

    public void setLeftMotor(double speed) {
      leftClimberMotor.set(speed);
    }

    public void setRightMotor(double speed) {
      rightClimberMotor.set(speed);
    }
}
