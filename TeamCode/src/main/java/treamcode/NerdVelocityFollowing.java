package treamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;

public class NerdVelocityFollowing {

    public boolean debugFlag = false;
    public boolean debugFlag2 = false;

    public static int LWM = 1;
    public static int RWM = 1;

    public static double accelRate = 0.1;
    public static double KV = 1;

    public static double DBanMax = 1;
    public static double DBanMin = -1;

    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;

    public static double FLError = 0;
    public static double FLTotalError = 0;
    public static double FLDError = 0;
    public static double FLSpeedCorr = 0;
    public static double FLPrevError = 0;

    public static double FRError = 0;
    public static double FRTotalError = 0;
    public static double FRDError = 0;
    public static double FRSpeedCorr = 0;
    public static double FRPrevError = 0;

    public static double RLError = 0;
    public static double RLTotalError = 0;
    public static double RLDError = 0;
    public static double RLSpeedCorr = 0;
    public static double RLPrevError = 0;

    public static double RRError = 0;
    public static double RRTotalError = 0;
    public static double RRDError = 0;
    public static double RRSpeedCorr = 0;
    public static double RRPrevError = 0;

    public static double maxPower = 0;

    public static double wheelDiameter = 3.54331; // For omni wheels we are using

    public static ArrayList<Double> rAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static ArrayList<Double> FLrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static ArrayList<Double> FRrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static ArrayList<Double> RLrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static ArrayList<Double> RRrAverage = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
    public static double sum = 0;
    public static double rAverageSize = 6;

    public static double [] velocityFollowing(double FLSpeedTarget, double RRSpeedTarget, double FRSpeedTarget,
                                              double RLSpeedTarget, double FLMotorSpeed, double RRMotorSpeed,
                                              double FRMotorSpeed, double RLMotorSpeed, double deltaTime) {


            double FLMtrSpd = PIDVelocFL(deltaTime, FLrunningAverage(FLMotorSpeed), LWM * rampFL(KV, accelRate, FLSpeedTarget, FLMotorSpeed));
            double FRMtrSpd = PIDVelocFR(deltaTime, FRrunningAverage(FRMotorSpeed), RWM * rampFR(KV, accelRate, FRSpeedTarget, FRMotorSpeed));
            double RLMtrSpd = PIDVelocRL(deltaTime, RLrunningAverage(RLMotorSpeed), LWM * rampRL(KV, accelRate, RLSpeedTarget, RLMotorSpeed));
            double RRMtrSpd = PIDVelocRR(deltaTime, RRrunningAverage(RRMotorSpeed), RWM * rampRR(KV, accelRate, RRSpeedTarget, RRMotorSpeed));

            double FLMtrPwr = speedToPower(FLSpeedTarget + FLMtrSpd);
            double FRMtrPwr = speedToPower(FRSpeedTarget + FRMtrSpd);
            double RLMtrPwr = speedToPower(RLSpeedTarget + RLMtrSpd);
            double RRMtrPwr = speedToPower(RRSpeedTarget + RRMtrSpd);

//        public void setDebug(boolean debugFlag2){this.debugFlag2=debugFlag2; }
//
//        if (debugFlag2) {
//            RobotLog.d("NerdVelocityFollowing - deltaTime %f, FLSpeedTarget %f, FLMtrSpd %f, FLMtrPwr %f, FRSpeedTarget %f, FRMtrSpd %f, FRMtrPwr %f, RLSpeedTarget %f, RLMtrSpd %f, RLMtrPwr %f, RRSpeedTarget %f, RRMtrSpd %f, RRMtrPwr %f",
//                    deltaTime, FLSpeedTarget, FLMtrSpd, FLMtrPwr, FRSpeedTarget, FRMtrSpd, FRMtrPwr, RLSpeedTarget, RLMtrSpd, RLMtrPwr, RRSpeedTarget, RRMtrSpd, RRMtrPwr);
//        }

        if(Math.abs(FLMtrPwr) > Math.abs(FRMtrPwr) && Math.abs(FLMtrPwr) > Math.abs(RLMtrPwr) && Math.abs(FLMtrPwr) > Math.abs(RRMtrPwr)){
            maxPower = Math.abs(FLMtrPwr);
        }
        else if (Math.abs(FRMtrPwr) > Math.abs(FLMtrPwr) && Math.abs(FRMtrPwr) > Math.abs(RLMtrPwr) && Math.abs(FRMtrPwr) > Math.abs(RRMtrPwr)){
            maxPower = Math.abs(FRMtrPwr);
        }
        else if(Math.abs(RRMtrPwr) > Math.abs(FRMtrPwr) && Math.abs(RRMtrPwr) > Math.abs(RLMtrPwr) && Math.abs(RRMtrPwr) > Math.abs(FLMtrPwr)){
            maxPower = Math.abs(RRMtrPwr);
        }
        if(Math.abs(RLMtrPwr) > Math.abs(FRMtrPwr) && Math.abs(RLMtrPwr) > Math.abs(FLMtrPwr) && Math.abs(RLMtrPwr) > Math.abs(RRMtrPwr)) {
            maxPower = Math.abs(RLMtrPwr);
        }

        if(maxPower > 1) {
            FLMtrPwr /= maxPower;
            FRMtrPwr /= maxPower;
            RLMtrPwr /= maxPower;
            RRMtrPwr /= maxPower;
        }

            double [] motorPowers = {FLMtrPwr, FRMtrPwr, RLMtrPwr, RRMtrPwr};
            return motorPowers;

        }

    private static double speedToPower(double motorSpeedInchesPerSec){

        double motorRPS = motorSpeedInchesPerSec / wheelDiameter / Math.PI; //convert motor power to ticks per second, 300 rpm max motor speed, 60 seconds in a minute
        double motorPower = motorRPS * 60 / 300; //60 seconds in a minute and 300 rpm motor max speed
        return motorPower;

    }

    public static double PIDVelocFL(double deltaTime, double FLCurrVeloc, double FLTVeloc) {

        //calculate error (Proportional)
        FLError = FLTVeloc - FLCurrVeloc;
        //Calculate Total error (Integral)
        FLTotalError = (FLError * deltaTime) + FLTotalError;
        //do dead band
        if (DBanMax > FLError && FLError > DBanMin) {
            FLError = 0;
        }
        //calculate delta error (Derivative)
        FLDError = (FLError - FLPrevError) / deltaTime;
        FLSpeedCorr = (FLError * kP) + (FLTotalError * kI) + (FLDError * kD);
        FLPrevError = FLError;

        return FLSpeedCorr;
    }

    public static double PIDVelocFR(double deltaTime, double FRCurrVeloc, double FRTVeloc) {

        //calculate error (Proportional)
        FRError = FRTVeloc - FRCurrVeloc;
        //Calculate Total error (Integral)
        FRTotalError = (FRError * deltaTime) + FRTotalError;
        //do dead band
        if (DBanMax > FRError && FRError > DBanMin) {
            FRError = 0;
        }
        //calculate delta error (Derivative)
        FRDError = (FRError - FRPrevError) / deltaTime;
        FRSpeedCorr = (FRError * kP) + (FRTotalError * kI) + (FRDError * kD);
        FRPrevError = FRError;

        return FRSpeedCorr;
    }

    public static double PIDVelocRL(double deltaTime, double RLCurrVeloc, double RLTVeloc) {

        //calculate error (Proportional)
        RLError = RLTVeloc - RLCurrVeloc;
        //Calculate Total error (Integral)
        RLTotalError = (RLError * deltaTime) + RLTotalError;
        //do dead band
        if (DBanMax > RLError && RLError > DBanMin) {
            RLError = 0;
        }
        //calculate delta error (Derivative)
        RLDError = (RLError - RLPrevError) / deltaTime;
        RLSpeedCorr = (RLError * kP) + (RLTotalError * kI) + (RLDError * kD);
        RLPrevError = RLError;

        return RLSpeedCorr;
    }

    public static double PIDVelocRR(double deltaTime, double RRCurrVeloc, double RRTVeloc) {

        //calculate error (Proportional)
        RRError = RRTVeloc - RRCurrVeloc;
        //Calculate Total error (Integral)
        RRTotalError = (RRError * deltaTime) + RRTotalError;
        //do dead band
        if (DBanMax > RRError && RRError > DBanMin) {
            RRError = 0;
        }
        //calculate delta error (Derivative)
        RRDError = (RRError - RRPrevError) / deltaTime;
        RRSpeedCorr = (RRError * kP) + (RRTotalError * kI) + (RRDError * kD);
        RRPrevError = RRError;

        return RRSpeedCorr;
    }

    public static double FLrunningAverage(double inputValueFL) {


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
    public static double FRrunningAverage(double inputValueFR) {


        FRrAverage.add(inputValueFR);

        if (rAverage.size() > rAverageSize)
            FRrAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += FRrAverage.get(i);

//        opmode.telemetry.addData("sumFR", sum);


        //return inputValue;
        return sum/FRrAverage.size();


    }
    public static double RLrunningAverage(double inputValueRL) {


        RLrAverage.add(inputValueRL);

        if (rAverage.size() > rAverageSize)
            RLrAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += RLrAverage.get(i);

//        opmode.telemetry.addData("sumRL", sum);


        //return inputValue;
        return sum/RLrAverage.size();


    }
    public static double RRrunningAverage(double inputValueRR) {


        RRrAverage.add(inputValueRR);

        if (rAverage.size() > rAverageSize)
            RRrAverage.remove(0);

        sum = 0;
        for(int i = 0; i < rAverage.size(); i++)
            sum += RRrAverage.get(i);

//        opmode.telemetry.addData("sumRR", sum);


        //return inputValue;
        return sum/RRrAverage.size();


    }

    public static int rampFL(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }public static int rampFR(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }public static int rampRL(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }public static int rampRR(double KVeloc, double accelRate, double tarVeloc, double currVeloc) {

        return (int) Math.round(KVeloc*tarVeloc-(accelRate*((KVeloc*tarVeloc)-currVeloc)));

    }

}
