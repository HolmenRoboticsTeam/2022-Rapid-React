package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterFlywheelSubsystem extends SubsystemBase{
  private WPI_TalonFX shooterMotorFront = new WPI_TalonFX(ShooterConstants.kShooterFlywheelFrontMotorPort);
  private WPI_TalonFX shooterMotorBack = new WPI_TalonFX(ShooterConstants.kShooterFlywheelBackMotorPort);
  private BangBangController controller = new BangBangController();
  private SlewRateLimiter rateLimiter = new SlewRateLimiter(12);
  private SlewRateLimiter rateLimiter2 = new SlewRateLimiter(12);
  private SlewRateLimiter rateLimiter3 = new SlewRateLimiter(0.15);
  private SlewRateLimiter rateLimiter4 = new SlewRateLimiter(0.15);


  //private PIDController flywheelFeedback = new PIDController(ShooterConstants.kShooterFlywheelKP, ShooterConstants.kShooterFlywheelKI, ShooterConstants.kShooterFlywheelKD);

  public ShooterFlywheelSubsystem() {
    shooterMotorFront.configFactoryDefault();
    shooterMotorBack.configFactoryDefault();
    shooterMotorBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooterMotorFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    shooterMotorBack.setNeutralMode(NeutralMode.Coast);
    shooterMotorFront.setNeutralMode(NeutralMode.Coast);

  }

  public double velocityOfFrontWheel (){
    double ticks = shooterMotorFront.getSelectedSensorVelocity();
    return nativeUnitsToVelocity(ticks);
  }
  public double velocityOfBackWheel (){
    double ticks = shooterMotorBack.getSelectedSensorVelocity();
    return nativeUnitsToVelocity(ticks);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Front Flywheel Speed", velocityOfFrontWheel());
    SmartDashboard.putNumber("Shooter Back Flywheel Speed",velocityOfBackWheel());
  }

  // PID stuff
  public void setFrontMotorVelocity(double velocity) {
    double output = controller.calculate(this.rateLimiter.calculate(velocityOfFrontWheel()), velocity);
    output = this.rateLimiter3.calculate(output);
    System.out.println(this.rateLimiter.calculate(velocityOfFrontWheel()));
    SmartDashboard.putNumber("Shooter Front PID", output);
    shooterMotorFront.set(output);
  }

  public void setBackMotorVelocity(double velocity) {
    double output = controller.calculate(this.rateLimiter2.calculate(velocityOfBackWheel()), velocity);
    output = this.rateLimiter4.calculate(output);
    SmartDashboard.putNumber("Shooter Back PID", output);
    shooterMotorBack.set(output);
  }
  private double nativeUnitsToVelocity(double sensorCountsPer100Ms) {
    double motorRotationsPer100ms = sensorCountsPer100Ms / 2048.0;
    double motorRotationsPerSecond = motorRotationsPer100ms * 10.0;
    double wheelRotationsPerSecond = motorRotationsPerSecond / 4.0;
    double velocityMetersPerSecond = wheelRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(3.0);
    return velocityMetersPerSecond;
  }

  public void setMotor(double speed){
    shooterMotorFront.set(speed);
    shooterMotorBack.set(speed);
  }
}
