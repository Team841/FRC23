package frc.lib.actuatuors;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class BioFalcon extends TalonFX {

    int timeOutMs = 0;
    int PIDLoopIdx = 0;
    Gains BioGains;

    double gearRatio = 1.0;
    double countsPerRev = 2048.0;

    /**
     * Creates a wrapper classed TalonFX
     * @param CANid The CANid the Talon is on
     * @param brakeMode Starting NeutralMode of the Talon
     */
    public BioFalcon(int CANid, boolean brakeMode){
        super(CANid);
        this.configFactoryDefault();
        bandWithLimitMotorCAN(this);

        setBrakeMode(brakeMode);
    }

    /**
     * Creates a wrapper classed Talon configuring a PID loop
     * @param CANid The CANid the Talon is on
     * @param brakeMode Starting NeutralMode of the Talon
     * @param gearRatioIn Gear Ratio of the system the Falcon is connected to
     * @param InputGains The (Gains) PID values for the Talon, should be set in constants
     * @param maxOutput the Max output
     * @param minOutput the Min output
     * @param kTimeoutMs Timeout in Ms
     * @param kPIDLoopIdx Default as 0
     */
    public BioFalcon(int CANid, boolean brakeMode, double gearRatioIn, Gains InputGains, double maxOutput, double minOutput, int kTimeoutMs,  int kPIDLoopIdx){
        super(CANid);
        this.configFactoryDefault();

        /* Config CAN Bandwith*/
        bandWithLimitMotorCAN(this);

        gearRatio = gearRatioIn;
        BioGains = InputGains;
        timeOutMs = kTimeoutMs;
        PIDLoopIdx = kPIDLoopIdx;

        /* Config Built in PID */
        configPID(minOutput, maxOutput);

        setBrakeMode(brakeMode);
    }

    /**
     * Creates a BioFalcon using PID loops and Motion Magic
     * @param CANid The CANid the Talon is on
     * @param brakeMode Starting NeutralMode of the Talon
     * @param gearRatioIn Gear Ratio of the system the falcon is connected to
     * @param InputGains The (Gains) PID values for the Talon, should be set in constants
     * @param maxOutput The Max output
     * @param minOutput The Min output
     * @param kTimeoutMs Timeout in MS
     * @param kPIDLoopIdx Default as 0
     * @param cruiseUnitsPer100ms The cruise speed for motion magic
     * @param AccelUnitsPer100ms The accel rate for motion magic
     */
    public BioFalcon(int CANid, boolean brakeMode, double gearRatioIn, Gains InputGains, double maxOutput, double minOutput, int kTimeoutMs, int kPIDLoopIdx, int cruiseUnitsPer100ms, int AccelUnitsPer100ms){
        super(CANid);
        this.configFactoryDefault();

        /* Limit CAN Bandwith */
        bandWithLimitMotorCAN(this);

        gearRatio = gearRatioIn;
        BioGains = InputGains;
        timeOutMs = kTimeoutMs;
        PIDLoopIdx = kPIDLoopIdx;

        /* Config Built in PID */
        configPID(minOutput, maxOutput);

        /* Set Motion Magic Cruise Velocity and Acceleration */
        this.configMotionCruiseVelocity(cruiseUnitsPer100ms, kTimeoutMs);
        this.configMotionAcceleration(AccelUnitsPer100ms, kTimeoutMs);

        setBrakeMode(brakeMode);
    }

    public void configPID(double minOutput, double maxOutput){
        /* Config Built in PID */
        this.config_kP(PIDLoopIdx, BioGains.kP, timeOutMs);
        this.config_kI(PIDLoopIdx, BioGains.kI, timeOutMs);
        this.config_kD(PIDLoopIdx, BioGains.kD, timeOutMs);
        this.config_kF(PIDLoopIdx, BioGains.kF, timeOutMs);
        this.config_IntegralZone(PIDLoopIdx, BioGains.kIzone, timeOutMs);
        this.configPeakOutputForward(maxOutput, timeOutMs);
        this.configPeakOutputReverse(minOutput, timeOutMs);
        this.configPeakOutputForward(maxOutput, timeOutMs);
        this.configPeakOutputReverse(minOutput, timeOutMs);

    }

    public void setUp2023SuperstructureSettings(){
        this.defaultConfig();
        this.configIntegratedSensor(true);
    }

    public void configIntegratedSensor(boolean kSensorPhase) {
        this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PIDLoopIdx, timeOutMs);
        this.setSensorPhase(kSensorPhase);

    }

    public void defaultConfig(){
        this.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    }

    public void setBrakeMode(boolean brake){
        if(brake){
            this.setNeutralMode(NeutralMode.Brake);
        }else{
            this.setNeutralMode(NeutralMode.Coast);
        }

    }

    /* Encoder Methods */
    public double getDegrees(){
        return this.getSelectedSensorPosition() * gearRatio / countsPerRev  * 360.0;
    }

    public double getCounts(){
        return this.getSelectedSensorPosition();
    }


    /**
     * Bandwith config motor CAN for TalonFX Motor
     * @param motor
     */
    private void bandWithLimitMotorCAN(TalonFX motor) {

        motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,255);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General,40);
        motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        motor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer,255);
        motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,255);
        motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,255);
        motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,255);
        motor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,255);
    }
}
