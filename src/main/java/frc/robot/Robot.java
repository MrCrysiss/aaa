package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class Robot extends TimedRobot {
  double angle = 0.45;
  private static final int deviceID = 4;
  CANSparkMax motor;
  private RelativeEncoder m_encoder;
  Joystick m_stick = new Joystick(0);
  private final ArmFeedforward feedforward = new ArmFeedforward(0.26, 0.16, 4.36, 0.28);
  private final PIDController pid = new PIDController(0.24, 0, 0);

  @Override
  public void robotInit() {
    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_encoder = motor.getEncoder();

    m_encoder.setPositionConversionFactor(1 / 36);
    m_encoder.setVelocityConversionFactor(1 / 36);
    m_encoder.setPosition(0);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  
  }
  
  @Override
  public void robotPeriodic() {
    
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    
  }
  
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    if(m_stick.getRawButtonPressed(1)) {
    this.angle = 0.75;
    }else if (m_stick.getRawButtonPressed(2)) {
    this.angle = 0.20;
    }
   
    double res_ff = feedforward.calculate(angle, 0.3 , 0.1);
    double res_pid = pid.calculate(m_encoder.getPosition(), angle);
    motor.setVoltage(res_ff + res_pid);
  }
}