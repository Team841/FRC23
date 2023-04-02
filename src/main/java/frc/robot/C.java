// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class C {

    public static final class OI{
        public static final int driverPortLeft = 0; //controller USB port 0
        public static final int driverPortRight = 1; //controller USB port 1
        public static final int codriverPort = 2; //controller USB port 2
    }

    public static final class CANid{
        public static final int driveRight1 = 1;
        public static final int driveRight2 = 2; 
        public static final int driveLeft1 = 3;
        public static final int driveLeft2 = 4;

        public static final int shoulderMotor_Starboard = 9;
        public static final int shoulderMotor_Port = 10;
        public static final int elbowMotor = 11;

        public static final int IntakeNeo = 6;
        public static final int IntakeTalon = 4;
    }

    public static final class Drivestyle{
        //Drive Style definition, loop up. Don't change this! This is needed for the subsystem.
        public static final int tankdrive = 1;
        public static final int chezydrive = 2;

        //Drive. We can change this.
        public static int drivestyle = chezydrive;
        public static int invert = -1;

        public static double slowModeScaleFactor = 0.25;

        //Tuning the Chezy Drive - deadband, sensitivity & tolerancing values on raw joystick inputs
        public static final double throttleDeadband = 0.02; 
        public static final double wheelDeadband = 0.02;	
        public static final double sensitivityHigh = 0.5;	
        public static final double sensitivityLow = 0.5; // (**deafult uses low gear**)


        public static final double wheelNonLinearityHighGear = 0.5; //Chezy Drive non-linearity
        public static final double wheelNonLinearityLowGear = 0.6; //Chezy Drive non-linearity  (**deafult uses low gear**)

        public static final double QuickTurnSensitivityHigh = 0.005; //Chezy Drive quick turn sensitivity
        public static final double QuickTurnSensitivityLow = 0.005; //Chezy Drive quick turn sensitivity  (**deafult uses low gear**)

    }

    public static final class Drive{
        
        // Rev motor set current limit at the number of amps
        public static int currentLimit = 60;

        //Physical setup of the drive
        public static double gearRatio = 1/7.23;
        public static double wheelDiameter = 6.06;
        public static double countsPerRev = 42; // this is the neo brushless
    
        //PID constants for turning, specifically for AutoTurn command//
        public static final double turn_kp = 0.018275; 
        public static final double turn_ki = 0;
        public static final double turn_kd = 0.00175;
        public static final double turn_tolerance = 0.4;
        public static final double turn_velocity_tolerance = 0.025; 
        public static final double antiWindUp = 0.1;
 
        //PID constants for AutoDriveToDistance command
        public static final double distance_kp = 0.040;   
        public static final double distance_ki = 0.0001; 
        public static final double distance_kd = 1;     
        public static final double distance_kff = 0;
        public static final double distance_tolerance = 2; // in revolutions
        public static final double distance_kIz = 2; // in revolutions
        
        //PID constants for AutoBalance command
        public static final double balance_kp = 0.011;
        public static final double balance_ki = 0.01;
        public static final double balance_kd = 0; 

        public static final double balance_point = 0; //0 degrees

        public static final int Brake = 6;
        public static final int Brake_b = 7;
    }
    
    public static final class Superstructure{
        public static final double shoulder_kp = 0.0003;
        public static final double shoulder_ki = 0;
        public static final double shoulder_kd = 0;
        public static final double shoulder_kff = 0;
        public static final double shoulder_kIz = 0 ;
        public static final double shoulder_tolerance = 5; //in degrees
        public static final double shoulder_maxOutput = 1;
        public static final double shoulder_minOutput = -1;
        // public static final double 

        public static final double elbow_kp = 0.0005;
        public static final double elbow_ki = 0;
        public static final double elbow_kd = 0;
        public static final double elbow_kff = 0;
        public static final double elbow_kIz = 0;
        public static final double elbow_tolerance = 10;//in degrees
        public static final double elbow_maxOutput = 1;
        public static final double elbow_minOutput = -1;

        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public static final boolean kSensorPhase = true;

        public static final double shoulderGearRatio = 0.0037; // 1/240;
        public static final double elbowGearRatio = 0.0033333; // 1/300;
        public static final double countsPerRev = 2048;

        public static final double IntakeMotorTalonPercentPower = 0.4;

        public static final double IntakeCubeCThresh = 10;
        public static final double IntakeConeCThresh = 25;

        public static final int Elbow_Index_Channel = 1;
        public static final int Shoulder_Index_Channel = 0;


        public static final double offset_incrment_constant = 0.06;// 1 deg a second



        public static final double testMove = 0.03;
        public static final double shoulderAccelerationLimitv5 = 5000; //native sensor units per 100 ms per ms
        public static final double elbowAccelerationLimitv5 = 5000000; //native sensor units per 100 ms per ms
        public static final double shoulderVelocityLimitv5 = 5000; //native sensor units per 100ms, 1k is approx 10%
        public static final double elbowVelocityLimitv5 = 50000000;
        public static final int shoulderSCurveStrengthv5 = 0; // zero is trapizodal motion profiling. 
        public static final int elbowSCurveStrengthv5 = 0; //  zero is trapizodal motion profiling. 



        public static class StateMachinePositions{
            /* {Should_angle, Elbow_angle} */
            public static final double[] Home = {0.0,0.0};
            public static final double[] coDriverMode = {0.0,0.0};
            public static final double[] ExtendOut = {51.0,0.0};
            public static final double[] ExtendIn = {51.0,0.0};
            public static final double[] PreExtractIn = {149.3,-41};
            public static final double[] MidScoreCube = {61.9,-12.2};
            public static final double[] TopScoreCube = {163.5, -161.4};
            public static final double[] TopSCoreCone = {163.5, -161.4};
            public static final double[] MidScoreCone = {122.9 , -123.5};
            // public static final double[] GroundCone = {39.8,-75.2};
            public static final double[] Ground = {39.8,-88.2};  
            public static final double[] Pickup = {0.0,0.0};  //this isn't used
        }
    }
}