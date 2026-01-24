// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.SwerveConstants;
public class SwerveModule {
  private static final double kWheelRadius = SwerveConstants.kWheelDiameterMeters/2;
  private static final int kEncoderResolution =(int)SwerveConstants.kAngleEncoderResolution;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
 private final SparkMaxConfig m_driveMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_turningMotorConfig = new SparkMaxConfig();
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final CANcoder m_CANcoder;
  

  private final SparkClosedLoopController m_driveClosedLoopController;
  private final SparkClosedLoopController m_turnClosedLoopController;
private double m_moduleEncoderAngularOffset;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(SwerveConstants.driveGainP, SwerveConstants.driveGainI, SwerveConstants.driveGainD);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveConstants.turnGainP,
          SwerveConstants.turnGainI,
          SwerveConstants.turnGainD,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.5, 1.5);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.5, 0.25);
private int m_driveMotorChannel;
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoderChannel DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double moduleEncoderAngularOffset
      ) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    m_driveMotorConfig.encoder
      .positionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor)
      .velocityConversionFactor(SwerveConstants.kDrivingEncoderVelocityFactor);
    m_driveMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(SwerveConstants.driveGainP, SwerveConstants.driveGainI, SwerveConstants.driveGainD);
     m_driveMotor.configure(m_driveMotorConfig, ResetMode.valueOf("kNoResetSafeParameters"), PersistMode.valueOf("kPersistParameters"));

    m_driveMotorChannel = driveMotorChannel; //for debugging

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveClosedLoopController = m_driveMotor.getClosedLoopController();

     m_turningMotorConfig
      .inverted(SwerveConstants.kTurningEncoderInverted)
      .idleMode(IdleMode.kCoast);
    m_turningMotorConfig.encoder
      .positionConversionFactor(SwerveConstants.kTurningEncoderPositionFactor)
      .velocityConversionFactor(SwerveConstants.kTurningEncoderVelocityFactor);
    m_turningMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(SwerveConstants.turnGainP, SwerveConstants.turnGainI, SwerveConstants.turnGainD);
    //m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.configure(m_turningMotorConfig, ResetMode.valueOf("kNoResetSafeParameters"), PersistMode.valueOf("kPersistParameters"));

    m_turningEncoder = m_turningMotor.getEncoder();
    m_turnClosedLoopController = m_turningMotor.getClosedLoopController();
    m_CANcoder = new CANcoder(turningEncoderChannel);  // in radians    *360;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
   // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
     m_moduleEncoderAngularOffset = moduleEncoderAngularOffset*Math.PI*2;
    m_desiredState.angle = getAngle();
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
   return new SwerveModuleState(m_driveEncoder.getVelocity(), getAngle());
  }
  public void resetEncoders() {
      m_driveEncoder.setPosition(0);
      m_turningEncoder.setPosition(wheelAngle());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
     return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        getAngle());
  }
  public double getTurnAngle() {
    return m_turningEncoder.getPosition();
  }
  public double wheelAngle() {
    var angle = getCanCoder();
    double angleRad = angle.getRadians()-m_moduleEncoderAngularOffset;
    return angleRad;
  }
  public Rotation2d getCanCoder() {
    return Rotation2d.fromRadians(m_CANcoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(m_turningEncoder.getPosition());
  }
  public double TurnOutput() {
    double turn = m_CANcoder.getAbsolutePosition().getValueAsDouble();
    return turn;
  }

  public double DriveOutput() {
    double drive = m_driveEncoder.getVelocity();
    return drive;
  }
  
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(
            m_turningEncoder.getPosition(), desiredState.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
