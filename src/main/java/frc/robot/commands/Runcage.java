package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cageclimb;


    public class Runcage extends Command{
    private Cageclimb m_cage; 
    private DoubleSupplier m_run;
    public Runcage(Cageclimb subsystem, DoubleSupplier run) {
m_cage = subsystem;
m_run = run;
addRequirements(m_cage);
   
    }
    @Override
    public void execute() {
double Runvalue = m_run.getAsDouble();
Runvalue *= 0.1;
m_cage.Run(Runvalue);

    }
}
