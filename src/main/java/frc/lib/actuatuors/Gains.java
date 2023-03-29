/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.lib.actuatuors;

import com.ctre.phoenixpro.configs.*;

public class Gains {

    private boolean sparkMax = false;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kf = 0;
    private double kIzone = 0;
    private double kS = 0;
    private double kV = 0;
 
    public Gains(double _kP, double _kI, double _kD){
        kP = _kP;
        kI = _kI;
        kD = _kD;
    }

    // rev
    public Gains(boolean _sparkMax, double _kP, double _kI, double _kD, double _kf, double _kIzone){
        sparkMax = _sparkMax;
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kf = _kf;
        kIzone = _kIzone;
    }

    // Falcon Pro Gains
    public Gains(double _kP, double _kI, double _kD, double _ks, double _kV){
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kS = _ks;
        kV = _kV;
    }

    public Slot0Configs getSlot0(){
        Slot0Configs configs = new Slot0Configs();

        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        configs.kS = kS;
        configs.kV = kV;

        return configs;
    }

    public Slot1Configs getSlot1(){
        Slot1Configs configs = new Slot1Configs();

        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        configs.kS = kS;
        configs.kV = kV;

        return configs;
    } 

    public Slot2Configs getSlot2(){
        Slot2Configs configs = new Slot2Configs();

        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        configs.kS = kS;
        configs.kV = kV;

        return configs;
    }
}