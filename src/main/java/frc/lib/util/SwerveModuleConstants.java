package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    
    public  int driveMotorID;
    public  int angleMotorID;
    public  int canCoderID;
    public  Rotation2d angleOffset;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset){
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.canCoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
