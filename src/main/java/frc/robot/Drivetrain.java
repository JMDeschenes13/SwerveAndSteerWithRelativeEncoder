// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import edu.wpi.first.wpilibj.AnalogGyro;
import com.ctre.phoenix.sensors.PigeonIMU;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI/3; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(.368, 0.244);
  private final Translation2d m_frontRightLocation = new Translation2d(0.368, -0.244);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.368, 0.244);
  private final Translation2d m_backRightLocation = new Translation2d(-0.368, -0.244);

  public final SwerveModule m_frontLeft = new SwerveModule(
   DriveConstants.kFrontLeftDrivePort,
   DriveConstants.kFrontLeftTurningPort,
   DriveConstants.kFrontLeftEncoderChannelA,
   DriveConstants.kFrontLeftEncoderChannelB,
   DriveConstants.kfrontLeftDriveMotorInverted,
   DriveConstants.kfrontLeftTurningMotorInverted);

  public final SwerveModule m_frontRight = new SwerveModule(
   DriveConstants.kFrontRightDrivePort,
   DriveConstants.kFrontRightTuringPort,
   DriveConstants.kFrontRightEncoderChannelA,
   DriveConstants.kFrontRightEncoderChannelB,
   DriveConstants.kfrontRightDriveMotorInverted,
   DriveConstants.kfrontRightTurningMotorInverted);

  public final SwerveModule m_backLeft = new SwerveModule(
   DriveConstants.kBackLeftDrivePort,
   DriveConstants.kBackLeftTurningPort,
   DriveConstants.kBackLeftEncoderChannelA,
   DriveConstants.kBackLeftEncoderChannelB,
   DriveConstants.kbackLeftDriveMotorInverted,
   DriveConstants.kbackLeftTurningMotorInverted);
  public final SwerveModule m_backRight = new SwerveModule(
   DriveConstants.kBackRightDrivePort,
   DriveConstants.kBackRightTurningPort,
   DriveConstants.kBackRightEncoderChannelA,
   DriveConstants.kBackRightEncoderChannelB,
   DriveConstants.kbackRightDriveMotorInverted,
   DriveConstants.kbackRightTurningMotorInverted);

  private final PigeonIMU m_gyro = new PigeonIMU(32);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });
  public Rotation2d getRotation2d(){
    return new Rotation2d(Math.toRadians(m_gyro.getYaw()));
  }
  public Drivetrain() {
    m_gyro.setYaw(0);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    SmartDashboard.putNumber("Encoder 0 go to", swerveModuleStates[0].angle.getRadians());
    SmartDashboard.putNumber("Encoder 1 go to", swerveModuleStates[1].angle.getRadians());
    SmartDashboard.putNumber("Encoder 2 go to", swerveModuleStates[2].angle.getRadians());
    SmartDashboard.putNumber("Encoder 3 go to", swerveModuleStates[3].angle.getRadians());
  }

  @Override
  public void periodic(){
   
  }
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
