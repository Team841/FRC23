package frc.lib.acutators;

public class Gains{

    double kp, ki, kd, kff, kIz, tolerance, SMMV, SMMA;

    public Gains(double _kp, double _ki, double _kd, double _kff, double _kIz, double _tolerance){
        kp = _kp;
        ki = _ki;
        kd = _kd;
        kff = _kff;
        kIz = _kIz;
        tolerance = _tolerance;
    }

    public Gains(double _kp, double _ki, double _kd, double _kff, double _kIz, double _tolerance, double SMMV, double SMMA){
        kp = _kp;
        ki = _ki;
        kd = _kd;
        kff = _kff;
        kIz = _kIz;
        tolerance = _tolerance;
        SMMV = SMMV;
        SMMA = SMMA;
    }

    public Gains (double _kp, double _ki, double _kd){
        kp = _kp;
        ki = _ki;
        kd = _kd;
    }


    public double kp(){
        return kp;
    }

    public double ki(){
        return ki;
    }

    public double kd(){
        return kd;
    }

    public double kff(){
        return kff;
    }

    public double kIz(){
        return kIz;
    }

    public double tolerance(){
        return tolerance;
    }

    public double SMMV(){
        return SMMV;
    }

    public double SMMA(){
        return SMMA;
    }

}
