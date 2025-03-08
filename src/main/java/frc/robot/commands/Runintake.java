package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cageclimb;
import frc.robot.subsystems.Intake;


    public class Runintake extends Command{
    private Intake m_intake; 
    private DoubleSupplier m_run;
    public Runintake(Intake subsystem, DoubleSupplier run) {
m_intake = subsystem;
m_run = run;
addRequirements(m_intake);
   
    }
    @Override
    public void execute() {
double Runvalue = m_run.getAsDouble();
Runvalue *= 0.1;
m_intake.Run(Runvalue);

    }
}
