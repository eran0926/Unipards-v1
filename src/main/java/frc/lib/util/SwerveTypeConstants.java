package frc.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.lib.convert.Convertions;

public class SwerveTypeConstants {
    public  double wheelDiameter;
    public  double wheelCircumference;
    public  double angleGearRadio;
    public  double driveGearRadio;
    public  double anglePIDF[]; // [P, I, D]

    public boolean angleMotorInverted;
    public InvertedValue driveMotorInverted;
    public InvertedValue canCoderInverted;

    public SwerveTypeConstants(
        double wheelDiameter, double angleGearRadio, double driveGearRadio, 
        double anglePIDF[], 
        boolean angleMotorInverted, InvertedValue driveMotorInverted, InvertedValue canCoderInverted){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRadio = angleGearRadio;
        this.driveGearRadio = driveGearRadio;
        this.anglePIDF = anglePIDF;
        this.angleMotorInverted = angleMotorInverted;
        this.driveMotorInverted = driveMotorInverted;
        this.canCoderInverted = canCoderInverted;

    }
    public static SwerveTypeConstants SDSMK4I_L1(){
        double wheelDiameter = Convertions.inchesToMeters(4.0);

        double driveGearRadio = SDSMK4I_L1_DRIVEGEAR;
        double angleGearRadio = SDSMK4I_L1_ANGLEGEAR;

        InvertedValue driveMotorInverted = InvertedValue.Clockwise_Positive;
        boolean angleMotorInverted = true;
        InvertedValue canCoderInverted = InvertedValue.Clockwise_Positive;

        double anglePIDF[] = {0.01, 0.0, 0.0,0.0};
        return new SwerveTypeConstants(
            wheelDiameter, angleGearRadio,driveGearRadio,
            anglePIDF, 
            angleMotorInverted, driveMotorInverted, canCoderInverted);
    }

    public static final double SDSMK4I_L1_DRIVEGEAR = 8.14;
    public static final double SDSMK4I_L1_ANGLEGEAR = 150.0/7.0;
}
