package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Cageclimb extends SubsystemBase {

     SparkFlex m_cageSpark = new SparkFlex(CANIds.kTestcageMotor, MotorType.kBrushless);
   
     public Cageclimb() { }

     public void Run(double Runvalue){
          m_cageSpark.set(Runvalue);
     }
}
