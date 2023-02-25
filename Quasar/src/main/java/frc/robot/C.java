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
        public static final int kRB = 6; //button map
        public static final int kLT = 7;//button map
        public static final int kRT = 8;
        public static final int kB = 3;
        public static final int kA = 2;
        public static final int kLB = 5;
        public static final int kX = 1;
        public static final int kY = 4;
    }

    public static final class CANid{
        public static final int driveRight1 = 1;
        public static final int driveRight2 = 2; 
        public static final int driveLeft1 = 3;
        public static final int driveLeft2 = 4;
    }

    public static final class Drive{
        //Drive Style definition, loop up. Don't change this! This is needed for the subsystem.
        public static final int tankdrive = 1;
        public static final int chezydrive = 2;

        //Drive. We can change this.
        public static int drivestyle = chezydrive;
        public static int invert = -1;

        public static double slowModeScaleFactor = 0.25;

        // Rev motor set current limit at the number of amps
        public static int currentLimit = 60;

        //Physical setup of the drive
        public static double gearRatio = 1/7.23;
        public static double wheelDiameter = 6.06;
        public static double countsPerRev = 42; // this is the neo brushless

        //Tuning the Chezy Drive - deadband, sensitivity & tolerancing values on raw joystick inputs
        public static final double throttleDeadband = 0.02; 
        public static final double wheelDeadband = 0.02;	
        public static final double sensitivityHigh = 0.5;	
        public static final double sensitivityLow = 0.5;
    
        public static final double turn_kp = 0.019; 
        public static final double turn_ki = 0.001;
        public static final double turn_kd = 0.001;

        public static final double turn_tolerance = 1;
 
        public static final double distance_kp = 0.040;   
        public static final double distance_ki = 0.0001; 
        public static final double distance_kd = 1;     

        public static final double distance_kff = 0;

        public static final double distance_tolerance = 2; // in revolutions

        public static final double distance_kIz = 2; // in revolutions

    }
            

}
