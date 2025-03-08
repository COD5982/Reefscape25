package frc.robot.subsystems;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
public abstract class DriveTrainBase extends SubsystemBase {
    protected Field2d m_field = new Field2d();
    
public DriveTrainBase() {
  
  RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    } /* 
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    ); */
    
  SmartDashboard.putData("Field", m_field);
}


    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed *= getMaxSpeedMetersPerSecond();
        ySpeed *= getMaxSpeedMetersPerSecond();
        rot *= getMaxSpeedMetersPerSecond();
    var swerveModuleStates =
        getSwerveDriveKinematics().toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, this.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                    DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, getMaxSpeedMetersPerSecond());
    getFrontLeftModule().setDesiredState(swerveModuleStates[0]);
    getFrontRightModule().setDesiredState(swerveModuleStates[1]);
    getBackLeftModule().setDesiredState(swerveModuleStates[2]);
    getBackRightModule().setDesiredState(swerveModuleStates[3]);
  }
public void setStates(SwerveModuleState[] swerveModuleStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, getMaxSpeedMetersPerSecond());
  getFrontLeftModule().setDesiredState(swerveModuleStates[0]);
  getFrontRightModule().setDesiredState(swerveModuleStates[1]);
  getBackLeftModule().setDesiredState(swerveModuleStates[2]);
  getBackRightModule().setDesiredState(swerveModuleStates[3]);
}

  abstract double getMaxSpeedMetersPerSecond();
  abstract Rotation2d getRotation2d();
  abstract SwerveDriveKinematics getSwerveDriveKinematics();
  abstract ISwerveModule getFrontLeftModule();
  abstract ISwerveModule getFrontRightModule();
  abstract ISwerveModule getBackLeftModule();
  abstract ISwerveModule getBackRightModule();
  
}
