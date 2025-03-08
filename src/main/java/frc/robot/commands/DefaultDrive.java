package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainBase;

public class DefaultDrive extends Command {
    private final DriveTrainBase m_drive;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rotSpeed;

    public DefaultDrive(DriveTrainBase subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
        m_drive = subsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        addRequirements(m_drive);
     }

    @Override
    public void execute() {
        double xspeed=m_xSpeed.getAsDouble();
        if (Math.abs(xspeed)<0.1){xspeed=0;}
        double yspeed=m_ySpeed.getAsDouble();
        if (Math.abs(yspeed)<0.1){yspeed=0;}
        double rotspeed=m_rotSpeed.getAsDouble();
        if (Math.abs(rotspeed)<0.1){rotspeed=0;}
        m_drive.drive(xspeed, yspeed, rotspeed, true);
        
    }
}
