package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Intake extends SubsystemBase {

     SparkMax m_intake = new SparkMax(CANIds.kIntakeMotor, MotorType.kBrushed);
   
     public Intake() { }

     public void Run(double Runvalue){
          m_intake.set(Runvalue);
     }
         public Command RunCommand(double Runvalue){
        return this.runOnce(() -> this.Run(Runvalue));
    }
}
