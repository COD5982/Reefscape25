package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Lift extends SubsystemBase{
    //reverse the front
    SparkFlex m_frontSpark = new SparkFlex(CANIds.kLiftFrontMotor, MotorType.kBrushless);
    SparkFlex m_backSpark = new SparkFlex(CANIds.kLiftBackMotor, MotorType.kBrushless);
    SparkClosedLoopController m_controllerleft = m_frontSpark.getClosedLoopController();
    SparkClosedLoopController m_controllerright = m_backSpark.getClosedLoopController();
private double targetposition = 0;
public static final double positionFloor = 0;
public static final double positionL1 = 10;
public static final double positionL2 = 20;
public static final double positionL3 = 40;
public static final double positionL4 = 50;
public static final double positionNet = 60;


    public Lift() {
//ShuffleboardTab LiftTab = Shuffleboard.getTab("Lift");
//LiftTab.add("Lift Motor", m_liftSpark);
//LiftTab.addDouble("Turn PID SP" ,()-> m_controller. );
SparkFlexConfig front = new SparkFlexConfig();
SparkFlexConfig back = new SparkFlexConfig();

// Set PID gains
front.closedLoop
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(-0.25, 0.25);

    back.closedLoop
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(-0.25, 0.25);
   front.inverted(true); 
    m_frontSpark.configure(front, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_backSpark.configure(back, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }
private void LifttoPosition(double Position){
    m_controllerleft.setReference(Position, ControlType.kPosition);
    m_controllerright.setReference(Position, ControlType.kPosition);
    targetposition = Position;

}
public Command LifttopositionCommand(double Position){
    return this.runOnce(() -> this.LifttoPosition(Position));

}

    public void Nudge(double Nudgevalue){
targetposition += Nudgevalue;
targetposition = MathUtil.clamp(targetposition,0,60);
LifttoPosition(targetposition);

    }
}
