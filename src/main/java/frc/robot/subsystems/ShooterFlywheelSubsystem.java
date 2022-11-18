package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
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
  private SlewRateLimiter rateLimiter3 = new SlewRateLimiter(500);
  private SlewRateLimiter rateLimiter4 = new SlewRateLimiter(500);


  //private PIDController flywheelFeedback = new PIDController(ShooterConstants.kShooterFlywheelKP, ShooterConstants.kShooterFlywheelKI, ShooterConstants.kShooterFlywheelKD);

  public ShooterFlywheelSubsystem() {
    shooterMotorFront.configFactoryDefault();
    shooterMotorBack.configFactoryDefault();
    shooterMotorBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooterMotorFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    shooterMotorBack.setNeutralMode(NeutralMode.Coast);
    shooterMotorFront.setNeutralMode(NeutralMode.Coast);

    shooterMotorBack.config_kP(0, 0.075);
    shooterMotorBack.config_kI(0, 0);
    shooterMotorBack.config_kD(0, 0.75);
    shooterMotorBack.config_kF(0, 0.05);

    shooterMotorFront.config_kP(0, 0.075);
    shooterMotorFront.config_kI(0, 0);
    shooterMotorFront.config_kD(0, 0.75);
    shooterMotorFront.config_kF(0, 0.05);

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
    double nativeVelocity = velocityToNativeUnits(velocity);
    nativeVelocity = this.rateLimiter3.calculate(nativeVelocity);
    shooterMotorFront.set(TalonFXControlMode.Velocity, nativeVelocity);
    // double output = controller.calculate(this.rateLimiter.calculate(velocityOfFrontWheel()), velocity);
    // output = this.rateLimiter3.calculate(output);
    // // System.out.println(this.rateLimiter.calculate(velocityOfFrontWheel()));
    SmartDashboard.putNumber("Shooter Front PID", nativeVelocity);
    // // output = 1;
    // // shooterMotorFront.set(output);
  }

  public void setBackMotorVelocity(double velocity) {
    double nativeVelocity = velocityToNativeUnits(velocity);
    nativeVelocity = this.rateLimiter4.calculate(nativeVelocity);
    shooterMotorBack.set(TalonFXControlMode.Velocity, nativeVelocity);
    // double output = controller.calculate(this.rateLimiter2.calculate(velocityOfBackWheel()), velocity);
    // output = this.rateLimiter4.calculate(output);
    SmartDashboard.putNumber("Shooter Back PID", nativeVelocity);
    // // output = 1;
    // shooterMotorBack.set(output);
  }

  private double nativeUnitsToVelocity(double sensorCountsPer100Ms) {
    double motorRotationsPer100ms = sensorCountsPer100Ms / 2048.0;
    double motorRotationsPerSecond = motorRotationsPer100ms * 10.0;
    double wheelRotationsPerSecond = motorRotationsPerSecond / 1.0;
    double velocityMetersPerSecond = wheelRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(3.0);
    return velocityMetersPerSecond;
  }

  private int velocityToNativeUnits (double velocityMetersPerSecond) {
    double wheelRotationsPerSecond = velocityMetersPerSecond / (2 * Math.PI * Units.inchesToMeters(3.0));
    double motorRotationsPerSecond = wheelRotationsPerSecond / 1.0;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0;
    int sensorCountsPer100Ms = (int)(motorRotationsPer100ms * 2048.0);
    return sensorCountsPer100Ms;
  }

  public void setMotor(double speed){
    shooterMotorFront.set(speed);
    shooterMotorBack.set(speed);
  }
}
