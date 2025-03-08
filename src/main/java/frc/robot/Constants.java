// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final int kCopilotButtonBox = 2;
  }


  public static final class CANIds {
    public static final int kTestDriveTrainFrontLeftPower = 3;
    public static final int kTestDriveTrainFrontLeftTurn = 4;
    public static final int kTestDriveTrainFrontLeftEncoder = 21;
    public static final int kTestDriveTrainFrontRightPower = 5;
    public static final int kTestDriveTrainFrontRightTurn = 2;
    public static final int kTestDriveTrainFrontRightEncoder = 22;
    public static final int kTestDriveTrainBackLeftPower = 8;
    public static final int kTestDriveTrainBackLeftTurn = 9;
    public static final int kTestDriveTrainBackLeftEncoder = 24;
    public static final int kTestDriveTrainBackRightPower = 6;
    public static final int kTestDriveTrainBackRightTurn = 7;
    public static final int kTestDriveTrainBackRightEncoder = 23;
    public static final int kIntakeMotor = 17;
    public static final int kLiftFrontMotor = 18;
    public static final int kLiftBackMotor = 19;
    public static final int kTestarmMotor = 20;
    public static final int kTestcageMotor = 25;
  }

  public static final class DriveConstants {
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;
    

  }
  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    //public static final int kEncoderCPR = 1;
   // public static final double kWheelDiameterMeters = 0.15;
   // public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
    //    (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

   // public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
   //     (2 * Math.PI) / kEncoderCPR;
   public static final double kWheelDiameterInches=4;
   public static final double kWheelDiameterFeet=kWheelDiameterInches / 12.0;
   public static final double kWheelCircumfrenceFeet=kWheelDiameterFeet * Math.PI;
   public static final double kWheelCircumfrenceMeters=kWheelCircumfrenceFeet / 3.28;
   public static final double kMotorRPM2MPS=kWheelCircumfrenceMeters * (1 / 60.0);
   public static final double kDriveMotorticks2mps=kWheelCircumfrenceMeters * (1.0 / 8.14)*(1 / 60.0);
    public static final double kPModuleTurningController = 0.25;
    public static final double kDriveMotorConversionFactor = (1.0 / 8.14) * kWheelCircumfrenceMeters;
    public static final double kPModuleDriveController = 0.01;
    public static final double kIModuleDriveController = 0.00;
    public static final double kFFModuleDriveController = 0.225;
  }
public static final class Swerve{
  public static final PIDConstants translationConstants = new PIDConstants(0.5, 0.0, 0.0);
  public static final PIDConstants rotationConstants = new PIDConstants(0.5, 0.0, 0.0);

}
}
