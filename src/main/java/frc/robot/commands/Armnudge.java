package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrainBase;
import frc.robot.subsystems.Lift;

public class Armnudge extends Command{
    private Arm m_Arm; 
    private DoubleSupplier m_Nudge;
    public Armnudge(Arm subsystem, DoubleSupplier nudge) {
m_Arm = subsystem;
m_Nudge = nudge;
addRequirements(m_Arm);
   
    }
    @Override
    public void execute() {
double Nudgevalue = m_Nudge.getAsDouble();
Nudgevalue *= 0.1;
m_Arm.Nudge(Nudgevalue);

    }
}