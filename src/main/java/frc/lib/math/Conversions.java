package frc.lib.math;

public class Conversions {

    public double inchesToRotationsDrive(double inches, double gearRatio, double wheelDiameter){
        return (inches / (wheelDiameter * Math.PI)) / gearRatio;
    }
}
