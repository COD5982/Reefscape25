package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

public class DriveTrain extends DriveTrainBase {

  private static final double kMaxSpeedMetersPerSecond = 3.2; // 4.0;
  private AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private final Translation2d m_frontLeftLocation = new Translation2d(0.267, 0.311);
  private final Translation2d m_frontRightLocation = new Translation2d(0.267, -0.311);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.267, 0.311);
  private final Translation2d m_backRightLocation = new Translation2d(-0.267, -0.311);

  private final SwerveDriveKinematics m_kinematics =
    new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveModule m_frontLeft = new SwerveModule(
    CANIds.kTestDriveTrainFrontLeftPower,
    CANIds.kTestDriveTrainFrontLeftTurn,
    CANIds.kTestDriveTrainFrontLeftEncoder);

  private final SwerveModule m_frontRight = new SwerveModule(
    CANIds.kTestDriveTrainFrontRightPower,
    CANIds.kTestDriveTrainFrontRightTurn,
    CANIds.kTestDriveTrainFrontRightEncoder);

  private final SwerveModule m_backLeft = new SwerveModule(
    CANIds.kTestDriveTrainBackLeftPower,
    CANIds.kTestDriveTrainBackLeftTurn,
    CANIds.kTestDriveTrainBackLeftEncoder);

  private final SwerveModule m_backRight = new SwerveModule(
    CANIds.kTestDriveTrainBackRightPower,
    CANIds.kTestDriveTrainBackRightTurn,
    CANIds.kTestDriveTrainBackRightEncoder);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    m_kinematics,
    getRotation2d(),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()});

  public DriveTrain(){
    
    ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
    SwerveTab.add("Gyro", m_gyro);

    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          Constants.Swerve.translationConstants,
          Constants.Swerve.rotationConstants
        ),
        config,
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
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
  }

  public void periodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());

    Pose2d pose = m_odometry.getPoseMeters();

    SmartDashboard.putNumber("Odometry X (m)", pose.getX());
    SmartDashboard.putNumber("Odometry Y (m)", pose.getY());
    SmartDashboard.putNumber("Odometry Rotation (deg)", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Gyro Angle (deg)", m_gyro.getAngle());
  }

  @Override
  public void updateOdometry() {
    m_odometry.update(
      getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()});
  }
    
  @Override
  public Rotation2d getRotation2d(){ 
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  @Override
  SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_kinematics;
  }
  
  @Override
  SwerveModule getFrontLeftModule() {
    return m_frontLeft;
  }

  @Override
  SwerveModule getFrontRightModule() {    
    return m_frontRight;
  }

  @Override
  SwerveModule getBackLeftModule() {    
    return m_backLeft;
  }

  @Override
  SwerveModule getBackRightModule() {   
    return m_backRight;
  }

  @Override
  double getMaxSpeedMetersPerSecond() {
    return kMaxSpeedMetersPerSecond;
  }
    
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] state = {
      getFrontLeftModule().getState(),
      getFrontRightModule().getState(),
      getBackLeftModule().getState(),
      getBackRightModule().getState()};

    return state;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = {
      getFrontLeftModule().getPosition(),
      getFrontRightModule().getPosition(),
      getBackLeftModule().getPosition(),
      getBackRightModule().getPosition()};
    
      return positions;
  }

  @Override
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }
  
  @Override
  public void resetPose(Pose2d Pose){
    m_odometry.resetPosition(m_gyro.getRotation2d(), getPositions(), Pose);
  }
          
  public ChassisSpeeds getSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }
}
