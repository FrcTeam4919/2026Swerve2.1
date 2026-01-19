// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.util.sendable.Sendable;
import com.studica.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControlSystem;
import frc.robot.Constants.DriveConstants;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = DriveConstants.kMaxSpeed; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(18.5), Units.inchesToMeters(18.5));
  private final Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(18.5), -Units.inchesToMeters(18.5));
  private final Translation2d m_backLeftLocation = new Translation2d(-Units.inchesToMeters(18.5), Units.inchesToMeters(18.5));
  private final Translation2d m_backRightLocation = new Translation2d(-Units.inchesToMeters(18.5), -Units.inchesToMeters(18.5));
  
  private final SwerveModule m_frontLeft = new SwerveModule(ControlSystem.kLeftFrontDrive, ControlSystem.kLeftFrontTurn, ControlSystem.kLFturn, DriveConstants.kFrontLeftModuleAngularOffset);
  private final SwerveModule m_frontRight = new SwerveModule(ControlSystem.kRightFrontDrive, ControlSystem.kRightFrontTurn, ControlSystem.kRFturn, DriveConstants.kFrontRightModuleAngularOffset);
  private final SwerveModule m_backLeft = new SwerveModule(ControlSystem.kLeftBackDrive, ControlSystem.kLeftBackTurn, ControlSystem.kLBturn, DriveConstants.kBackLeftModuleAngularOffset);
  private final SwerveModule m_backRight = new SwerveModule(ControlSystem.kRightBackDrive, ControlSystem.kRightBackTurn, ControlSystem.kRBturn, DriveConstants.kBackRightModuleAngularOffset);

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);;

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          new Rotation2d(-m_gyro.getAngle()*Math.PI/180),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          Pose2d.kZero,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_poseEstimator.getEstimatedPosition().getRotation())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        new Rotation2d(-m_gyro.getAngle()*Math.PI/180),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
     
    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    m_poseEstimator.addVisionMeasurement(
        ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
            m_poseEstimator.getEstimatedPosition()),
        Timer.getTimestamp() - 0.3);
  }
  // Put values to SmartDashboard 
   public void SmartDashData(){
System.out.println("working smartdashboardcall");//for debugging
  SmartDashboard.putNumber("Front Left drive speed", m_frontLeft.DriveOutput());
  SmartDashboard.putNumber("Front Right drive speed", m_frontRight.DriveOutput());
  SmartDashboard.putNumber("Back Left drive speed", m_backLeft.DriveOutput());
  SmartDashboard.putNumber("Back Right drive speed", m_backRight.DriveOutput());
//Display Kinematics
   SmartDashboard.putNumber("Front Left encoder count", m_frontLeft.TurnOutput());
  SmartDashboard.putNumber("Front Right encoder count", m_frontRight.TurnOutput());
  SmartDashboard.putNumber("Back Left encoder count", m_backLeft.TurnOutput());
  SmartDashboard.putNumber("Back Right encoder count", m_backRight.TurnOutput());
 //Display Wheel orientations
  SmartDashboard.putNumber("Front Left wheel angle", m_frontLeft.wheelAngle());
  SmartDashboard.putNumber("Front Right wheel angle", m_frontRight.wheelAngle());
  SmartDashboard.putNumber("Back Left wheel angle", m_backLeft.wheelAngle());
  SmartDashboard.putNumber("Back Right wheel angle", m_backRight.wheelAngle());
 //Display Wheel orientations
  SmartDashboard.putNumber("Front Left NEO wheel angle", m_frontLeft.getTurnAngle());
  SmartDashboard.putNumber("Front Right NEO wheel angle", m_frontRight.getTurnAngle());
  SmartDashboard.putNumber("Back Left Neo wheel angle", m_backLeft.getTurnAngle());
  SmartDashboard.putNumber("Back Right NEO wheel angle", m_backRight.getTurnAngle());



      }
}
