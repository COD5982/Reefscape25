package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
;
public interface ISwerveModule {
   public void setDesiredState(SwerveModuleState desiredState) ; 
    public SwerveModulePosition getPosition();
    public SwerveModuleState getState();
}
