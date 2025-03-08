package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainBase;
import frc.robot.subsystems.Lift;

public class Liftnudge extends Command{
    private Lift m_Lift; 
    private DoubleSupplier m_Nudge;
    public Liftnudge(Lift subsystem, DoubleSupplier nudge) {
m_Lift = subsystem;
m_Nudge = nudge;
addRequirements(m_Lift);
   
    }
    @Override
    public void execute() {
double Nudgevalue = m_Nudge.getAsDouble();
Nudgevalue *= 0.1;
m_Lift.Nudge(Nudgevalue);

    }
}
