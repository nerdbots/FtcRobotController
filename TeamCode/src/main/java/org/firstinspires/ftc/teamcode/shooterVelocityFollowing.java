package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;

public class shooterVelocityFollowing {

    private LinearOpMode opMode;
    private HardwareMap hardwareMap;


    private DcMotorEx shooter;

    private ElapsedTime velocityElapsedTime = new ElapsedTime();

    public boolean debugFlag = false;
    public boolean debugFlag2 = false;

    public static double accelRate = 0.025;
    public static double KV = 1;

    public static double DBanMax = 1;
    public static double DBanMin = -1;

    public static double kP = 0.1;
    public static double kI = 0.025;
    public static double kD = 0;
    public static double kF = 0.0;

    public static double error = 0;
    public static double totalError = 0;
    public static double dError = 0;
    public static double speedCorr = 0;
    public static double prevError = 0;
    public static double directError = 0;


    public static double maxPower = 0;


    double prevTickTime = 0;
    int prevLeft = 0, prevRight = 0, prevLeftB = 0, prevRightB = 0;
    private final double wheelDiameter = 3.54331; // For omni wheels we are using
    private final double wheelMountAngle = 45.0; //For current drivetrain
    private final double GEAR_RATIO = 20.0 / 15.0;  // Gear ratio
    private final double ticksPerRotation = 540.0; //For omni wheels we are using
    public double [] Velocities = new double[5];
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;

    public static ArrayList<Double> rAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static ArrayList<Double> FLrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static ArrayList<Double> FRrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static ArrayList<Double> RLrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static ArrayList<Double> RRrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public double sum = 0;
    public double rAverageSize = 6;

    public shooterVelocityFollowing(LinearOpMode opmode) {
        this.opMode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    private double [] getVelocityForCurrentLoop() {
        int current = shooter.getCurrentPosition();

        double deltaTickTime = velocityElapsedTime.seconds();
        velocityElapsedTime.reset();

        double inches = ticksToInches((int) current, wheelDiameter, wheelMountAngle);

        resetMotors();

        double shooterVelocity = inches / deltaTickTime;

        int leftDeltaTicks = current;
        Velocities[0] = shooterVelocity;
        Velocities[1] = deltaTickTime; // added delta time to be used in PID

        return Velocities;  //inches per second
    }

    public double velocityFollowing(double speedTarget) {
            double[] shooterVelocityTime = getVelocityForCurrentLoop();


            double mtrSpd = PIDVeloc(shooterVelocityTime[1], runningAverage(shooterVelocityTime[0]), ramp(KV, accelRate, speedTarget, shooterVelocityTime[0]));

            double mtrPwr = speedToPower(speedTarget + mtrSpd);

//        public void setDebug(boolean debugFlag2){this.debugFlag2=debugFlag2; }
//
//        if (debugFlag2) {
//            RobotLog.d("NerdVelocityFollowing - deltaTime %f, FLSpeedTarget %f, FLMtrSpd %f, FLMtrPwr %f, FRSpeedTarget %f, FRMtrSpd %f, FRMtrPwr %f, RLSpeedTarget %f, RLMtrSpd %f, RLMtrPwr %f, RRSpeedTarget %f, RRMtrSpd %f, RRMtrPwr %f",
//                    deltaTime, FLSpeedTarget, FLMtrSpd, FLMtrPwr, FRSpeedTarget, FRMtrSpd, FRMtrPwr, RLSpeedTarget, RLMtrSpd, RLMtrPwr, RRSpeedTarget, RRMtrSpd, RRMtrPwr);
//        }

//        if(Math.abs(FLMtrPwr) > Math.abs(FRMtrPwr) && Math.abs(FLMtrPwr) > Math.abs(RLMtrPwr) && Math.abs(FLMtrPwr) > Math.abs(RRMtrPwr)){
//            maxPower = Math.abs(FLMtrPwr);
//        }
//        else if (Math.abs(FRMtrPwr) > Math.abs(FLMtrPwr) && Math.abs(FRMtrPwr) > Math.abs(RLMtrPwr) && Math.abs(FRMtrPwr) > Math.abs(RRMtrPwr)){
//            maxPower = Math.abs(FRMtrPwr);
//        }
//        else if(Math.abs(RRMtrPwr) > Math.abs(FRMtrPwr) && Math.abs(RRMtrPwr) > Math.abs(RLMtrPwr) && Math.abs(RRMtrPwr) > Math.abs(FLMtrPwr)){
//            maxPower = Math.abs(RRMtrPwr);
//        }
//        if(Math.abs(RLMtrPwr) > Math.abs(FRMtrPwr) && Math.abs(RLMtrPwr) > Math.abs(FLMtrPwr) && Math.abs(RLMtrPwr) > Math.abs(RRMtrPwr)) {
//            maxPower = Math.abs(RLMtrPwr);
//        }

//        if(maxPower > 1) {
//            FLMtrPwr /= maxPower;
//            FRMtrPwr /= maxPower;
//            RLMtrPwr /= maxPower;
//            RRMtrPwr /= maxPower;
//        }

            double motorPower = mtrPwr;
            return motorPower;

        }

    private double speedToPower(double motorSpeedInchesPerSec){

        double motorRPS = motorSpeedInchesPerSec / wheelDiameter / Math.PI; //convert motor power to ticks per second, 300 rpm max motor speed, 60 seconds in a minute
        double motorPower = motorRPS * 60 / 300; //60 seconds in a minute and 300 rpm motor max speed
        return motorPower;

    }

    public double PIDVeloc(double deltaTime, double currVeloc, double targetVeloc) {

        //calculate error (Proportional)
        error = targetVeloc - currVeloc;
        //Calculate Total error (Integral)
        totalError = (error * deltaTime) + totalError;
        //do dead band
        if (DBanMax > error && error > DBanMin) {
            error = 0;
        }
        //calculate delta error (Derivative)
        dError = (error - prevError) / deltaTime;

        //Calculate feed-forward (F)
        directError = targetVeloc;

        speedCorr = (error * kP) + (totalError * kI) + (dError * kD) + (directError * kF);
        prevError = error;

        return speedCorr;
    }



    public double runningAverage(double inputValueFL) {


        FLrAverage.add(inputValueFL);

        if (rAverage.size() > rAverageSize)
            FLrAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += FLrAverage.get(i);

//        opmode.telemetry.addData("sumFL", sum);


        //return inputValue;
        return sum/FLrAverage.size();


    }


    public int ramp(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }

//    public static void resetI() {
//        FLTotalError = 0;
//        FRTotalError = 0;
//        RLTotalError = 0;
//        RRTotalError = 0;
//    }

    public void initialize() {
        this.shooter = this.hardwareMap.get(DcMotorEx.class, "Front");

    }

    public double ticksToInches(int ticks, double wheelDiameter, double wheelMountAngle) {
        double circum = wheelDiameter * Math.PI;
        double numberofWheelRotations = (double) ticks / ticksPerRotation;
        double wheelDistanceToTravel = numberofWheelRotations * circum;
//        double straightDistanceToTravel = wheelDistanceToTravel; // (Math.cos(Math.toRadians(wheelMountAngle)) * GEAR_RATIO);
        return wheelDistanceToTravel;
    }

    private void resetMotors() {
        this.shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        this.shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        this.shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

}
