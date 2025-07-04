// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Armnudge;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.Liftnudge;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cageclimb;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainBase m_driveTrain = new DriveTrain();
  private final Lift m_lift = new Lift();
  private final Arm m_arm = new Arm();
  private final Cageclimb m_cage = new Cageclimb();
  private final Intake m_intake = new Intake();
  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_copilotController = new CommandXboxController(OperatorConstants.kCopilotControllerPort);
  private final CommandJoystick m_copilotButtonbox = new CommandJoystick(2);
    
  
  VideoSink server;
  UsbCamera cam;
  UsbCamera cam1;

  //private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    cam = CameraServer.startAutomaticCapture(0);
    cam1 = CameraServer.startAutomaticCapture(1);
    server =  CameraServer.getServer();
    server.setSource(cam);
    cam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    cam1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    NamedCommands.registerCommand("ScoreReef", Autos.ScoreReef(m_lift, m_arm, m_intake));
    NamedCommands.registerCommand("PickupBall", Autos.PickupBall(m_lift, m_arm, m_intake));
    // Configure the trigger bindings
    configureBindings();

    // Autonomous chooser
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    SmartDashboard.putData("Reset Pose", Autos.ResetPose(m_driveTrain));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driveTrain.setDefaultCommand(new DefaultDrive(m_driveTrain,
      ()-> -Math.signum(m_driverController.getLeftY()) * Math.abs(Math.pow(MathUtil.applyDeadband(m_driverController.getLeftY(),0.1),3)),
      ()-> -Math.signum(m_driverController.getLeftX()) * Math.abs(Math.pow(MathUtil.applyDeadband(m_driverController.getLeftX(),0.1),3)),
      ()-> -Math.signum(m_driverController.getRightX()) * Math.abs(Math.pow(MathUtil.applyDeadband(m_driverController.getRightX(),0.1),3)),
      
      ()-> m_driverController.rightBumper().getAsBoolean()&&m_arm.ArmSafeTravel()&&m_lift.LiftSafeTravel()));
      // ()-> -Math.signum(m_driverController.getLeftY()) * Math.pow(m_driverController.getLeftY(),4),
      // ()-> -Math.signum(m_driverController.getLeftX()) * Math.pow(m_driverController.getLeftX(),4),
      // ()-> -Math.signum(m_driverController.getRightX()) * Math.pow(m_driverController.getRightX(),4)));
        
      // ()-> -Math.signum(m_driverController.getLeftY()) * Math.pow(m_driverController.getLeftY(),2),
      // ()-> -Math.signum(m_driverController.getLeftX()) * Math.pow(m_driverController.getLeftX(),2),
      // ()-> -Math.signum(m_driverController.getRightX()) * Math.pow(m_driverController.getRightX(),2)));

      // ()-> -m_driverController.getLeftY(),
      // ()-> -m_driverController.getLeftX(),
      // ()-> -m_driverController.getRightX());

      Runnable setBallCam = () -> server.setSource(cam);
      Runnable setBargeCam = () -> server.setSource(cam1);

      m_driverController.povLeft().onTrue(new InstantCommand(setBallCam));
      m_driverController.povRight().onTrue(new InstantCommand(setBargeCam));
        
    // CONTROLLER CONTROLS
    m_copilotController.povUp().whileTrue(new Liftnudge(m_lift, ()->0.2 )); 
    m_copilotController.povDown().whileTrue(new Liftnudge(m_lift, ()->-0.2 ));
    m_copilotController.y().onTrue(m_arm.ArmtopositionCommand(Arm.Spitarm));
    m_copilotController.b().onTrue(m_arm.ArmtopositionCommand(Arm.Middlearm));
    m_copilotController.a().onTrue(m_arm.ArmtopositionCommand(Arm.Floorarm));
    m_copilotController.x().onTrue(m_arm.ArmtopositionCommand(Arm.Reefarm));
    m_copilotController.rightBumper().whileTrue(new RunCommand(() -> m_arm.Run(45), m_arm)).onFalse(new RunCommand(()-> m_arm.Run(0), m_arm));
    m_copilotController.leftBumper().whileTrue(new RunCommand(() -> m_arm.Run(-45), m_arm)).onFalse(new RunCommand(()-> m_arm.Run(0), m_arm));

    m_arm.setDefaultCommand(new Armnudge(m_arm, ()-> MathUtil.applyDeadband( m_copilotController.getRightY(),0.2)));
    m_cage.setDefaultCommand(new RunCommand(()->m_cage.Run(0), m_cage));

    m_intake.setDefaultCommand(new RunCommand(()->{
      double min = 0.001;
      double right = m_copilotController.getRightTriggerAxis();
      double left = m_copilotController.getLeftTriggerAxis();
      if (right > min){
        m_intake.Run(1.0);
      }
      else if (left > min){
        m_intake.Run(-1.0);
      }
      else{
        m_intake.Run(0.0);
      }
    }, m_intake));

    // BUTTON BOX CONTROLS
    m_copilotButtonbox.button(4).onTrue(m_lift.LifttopositionCommand(Lift.positionL1).alongWith(new InstantCommand(setBallCam)));
    m_copilotButtonbox.button(2).onTrue(m_lift.LifttopositionCommand(Lift.positionL2).alongWith(m_arm.ArmtopositionCommand(Arm.L2Arm)).alongWith(new InstantCommand(setBallCam)));
    m_copilotButtonbox.button(1).onTrue(m_lift.LifttopositionCommand(Lift.positionL3).alongWith(m_arm.ArmtopositionCommand(Arm.L3Arm)).alongWith(new InstantCommand(setBallCam)));
    m_copilotButtonbox.button(3).onTrue(m_lift.LifttopositionCommand(Lift.positionL4).alongWith(new InstantCommand(setBargeCam)));
    m_copilotButtonbox.button(5).onTrue(m_lift.LifttopositionCommand(Lift.positionNet).alongWith(new InstantCommand(setBargeCam)));
    m_copilotButtonbox.button(6).onTrue(m_lift.LifttopositionCommand(Lift.positionFloor).alongWith(new InstantCommand(setBallCam)));
    m_copilotButtonbox.button(8).onTrue(new RunCommand(()->m_cage.Run(-0.4), m_cage)).onFalse (new RunCommand(()->m_cage.Run(0), m_cage));
    m_copilotButtonbox.button(7).onTrue(new RunCommand(()->m_cage.Run(0.4), m_cage)).onFalse (new RunCommand(()->m_cage.Run(0), m_cage));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
