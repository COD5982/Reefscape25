package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class TestSwerveModule  implements ISwerveModule{
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private final CANcoder m_turnEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 
      ModuleConstants.kIModuleDriveController, 
      0
      );

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));


  public TestSwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderCanID) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turnEncoder = new CANcoder(turningEncoderCanID);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.inverted(false);
    m_turningMotor.configure(turnConfig, null, null);
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(false);

    m_driveMotor.configure(driveConfig, null, null);

ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
SwerveTab.add("Turn Encoder" + turningEncoderCanID, m_turnEncoder);
SwerveTab.addDouble("Turn PID SP" + turningEncoderCanID,()->m_turningPIDController.getSetpoint().position );
SwerveTab.addDouble("Turn PID Error" + turningEncoderCanID,()->m_turningPIDController.getPositionError() );
SwerveTab.addDouble("Turn Encoder Angle" + turningEncoderCanID,()-> m_turnEncoder.getAbsolutePosition().getValue().in(Radians));
SwerveTab.addDouble("Drive PID SP" + turningEncoderCanID,()->m_drivePIDController.getSetpoint() );
SwerveTab.addDouble("Drive PID Error" + turningEncoderCanID,()->m_drivePIDController.getPositionError() );
SwerveTab.addDouble("Drive Motor Volocity" + turningEncoderCanID, ()->m_driveMotor.getEncoder().getVelocity()* Constants.ModuleConstants.kDriveMotorticks2mps);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getEncoder().getVelocity()* Constants.ModuleConstants.kDriveMotorticks2mps, new Rotation2d(m_turnEncoder.getAbsolutePosition().getValue()));
        
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getEncoder().getPosition()* Constants.ModuleConstants.kDriveMotorConversionFactor, new Rotation2d((m_turnEncoder.getAbsolutePosition().getValue())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turnEncoder.getAbsolutePosition().getValue());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
     double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getEncoder().getVelocity() * Constants.ModuleConstants.kDriveMotorticks2mps, desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput =
        m_turningPIDController.calculate(
            m_turnEncoder.getAbsolutePosition().getValue().in(Radians), desiredState.angle.getRadians());
            driveOutput = driveOutput + ModuleConstants.kFFModuleDriveController * desiredState.speedMetersPerSecond;
//turnOutput=MathUtil.clamp(turnOutput, -0.5, 0.5);
//driveOutput=MathUtil.clamp(driveOutput, -0.5, 0.5);
    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.getEncoder().setPosition(0);
    m_turningMotor.getEncoder().setPosition(0);
    m_turnEncoder.setPosition(0);
  }
}