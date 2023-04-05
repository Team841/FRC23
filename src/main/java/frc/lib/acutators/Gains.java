package frc.lib.acutators;

public class Gains {

    double kP, kI, kD, kF, kIz, tolerance, SMMV, SMMA;


    public Gains(double _kP, double _kI, double _kd, double _kF, double _kIz, double _tolerance, double _SMMV, double _SMMA){
        kP = _kP;
        kI = _kI;
        kD = _kd;
        kF = _kF;
        kIz = _kIz;
        tolerance = _tolerance;
        SMMV = _SMMV;
        SMMA = _SMMA;
    }

    public Gains(double _kP, double _kI, double _kD){
        kP = _kP;
        kI = _kI;
        kD = _kD;
    }

    public double getTolerance(){
        return tolerance;
    }

    public double kP() { return kP; }

    public double kI() { return kI; }

    public double kD() { return kD; }

    public double kF() { return kF; }

    public double kIz() { return kIz; }

    public double SMMV() { return SMMV; }

    public double SMMA() { return SMMA; }


}
