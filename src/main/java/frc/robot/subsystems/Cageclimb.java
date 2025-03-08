package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Cageclimb extends SubsystemBase {
     SparkFlex m_cageSpark = new SparkFlex(CANIds.kTestcageMotor, MotorType.kBrushless);
   
public Cageclimb() {



}

public void Run(double Runvalue){
m_cageSpark.set(Runvalue);
}
}
