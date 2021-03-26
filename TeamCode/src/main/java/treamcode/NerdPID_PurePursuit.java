package treamcode;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

public class NerdPID_PurePursuit {

    public static double propError = 0;
    public static double intError = 0;
    public static double derError = 0;
    public static double prevDerError = 0;

    public static double propErrorD = 0;
    public static double intErrorD = 0;
    public static double derErrorD = 0;
    public static double prevDerErrorD = 0;

    public static double propErrorP = 0;
    public static double intErrorP = 0;
    public static double derErrorP = 0;
    public static double prevDerErrorP = 0;

    public static double propErrorMS = 0;
    public static double intErrorMS = 0;
    public static double derErrorMSInit = 0;
    public static double derErrorMS = 0;
    public static double prevDerErrorMS = 0;

    public static double propErrorSP = 0;
    public static double intErrorSP = 0;
    public static double derErrorSP = 0;

//    public static double zPowerStart = 0;
//    public static double zPowerIncrease = 0.075;


    public static boolean debugFlag=true;

//    public void setDebug(boolean debugFlag){
//        this.debugFlag=debugFlag;
//    }

    public static void resetIntError(){
        intError = 0;
        intErrorD = 0;
        intErrorMS = 0;
        intErrorP = 0;
        intErrorSP = 0;
//        zPowerStart = -0.35;

    }

    public static double zPower (double targetAngle, double gyroAngle, double deltaTimePID){

        double kP = 0.015; //0.050
        double kI = 0.003; //0.002
        double kD = 0.000; //0.000

        //calculate error (Proportional)
        propError = targetAngle - gyroAngle;

        //Calculate Total error (Integral)
         intError += (targetAngle - gyroAngle) * deltaTimePID;

//        //do deadban
//        if (DBanMax > error && error > DBanMin) {
//            error = 0;
//            //TotalError = 0;
//        }
        //calculate delta error (Derivative)
        derError = ((targetAngle - gyroAngle) - prevDerError) / deltaTimePID;
        prevDerError = derError;

        double zPowerPID = (propError * kP) + (intError * kI) + (derError * kD);

        return zPowerPID;

    }

    public static double zPowerDrive (double targetAngleDrive, double gyroAngleDrive, double deltaTimePIDDrive){

        double kPD = 0.005; //0.010
        double kID = 0.003; //0.005
        double kDD = 0.000; //0.000

        //calculate error (Proportional)
        propErrorD = targetAngleDrive - gyroAngleDrive;

        //Calculate Total error (Integral)
        intErrorD += (targetAngleDrive - gyroAngleDrive) * deltaTimePIDDrive;

//        //do deadban
//        if (DBanMax > error && error > DBanMin) {
//            error = 0;
//            //TotalError = 0;
//        }
        //calculate delta error (Derivative)
        derErrorD = ((targetAngleDrive - gyroAngleDrive) - prevDerErrorD) / deltaTimePIDDrive;
        prevDerErrorD = derErrorD;

//        double robotTurnSpeedFF = Range.clip((zPowerStart + zPowerIncrease), -0.5, 0);
//        zPowerStart = robotTurnSpeedFF;

        double zPowerPIDD = (propErrorD * kPD) + (intErrorD * kID) + (derErrorD * kDD);

        if (debugFlag) {
            RobotLog.d("zPowerDrive - deltaTime %f, targetAngleDrive %f, gyroAngleDrive %f, propErrorD %f, intErrorD %f, derErrorD %f, zPowerPIDD %f",
                    deltaTimePIDDrive, targetAngleDrive, gyroAngleDrive, propErrorD, intErrorD, derErrorD, zPowerPIDD);

        }

        return zPowerPIDD;

    }

    public static double zPowerPark (double targetAnglePark, double gyroAnglePark, double deltaTimePIDPark){

        double kPP = 0.005; //0.050
        double kIP = 0.002; //0.002
        double kDP = 0.000; //0.000

        //calculate error (Proportional)
        propErrorP = targetAnglePark - gyroAnglePark;

        //Calculate Total error (Integral)
        intErrorP += (targetAnglePark - gyroAnglePark) * deltaTimePIDPark;

//        //do deadban
//        if (DBanMax > error && error > DBanMin) {
//            error = 0;
//            //TotalError = 0;
//        }
        //calculate delta error (Derivative)
        derErrorP = ((targetAnglePark - gyroAnglePark) - prevDerErrorP) / deltaTimePIDPark;
        prevDerErrorP = derErrorP;

        double zPowerPIDP = (propErrorP * kPP) + (intErrorP * kIP) + (derErrorP * kDP);

        return zPowerPIDP;

    }

    public static double movementSpeedPID (double robotDistanceToTarget, double prevRobotDistanceToTarget, double deltaTimePIDMS){
        double kPMS = 0.033; //old used to be 0.033?
        double kIMS = 0.250;  // old used to be 0.020
        double kDMS = 0.017; //competition bot = 0.020?

        //calculate error (Proportional)
        propErrorMS = robotDistanceToTarget;

        //Calculate Total error (Integral)
        intErrorMS += (robotDistanceToTarget) * deltaTimePIDMS / 10;

        //Calculate change in error (Derivative)
        derErrorMS = (robotDistanceToTarget - prevRobotDistanceToTarget) / deltaTimePIDMS;

        double movementSpeedPIDPow = (propErrorMS * kPMS) + (intErrorMS * kIMS) + (derErrorMS * kDMS);

        if (debugFlag) {
            RobotLog.d("movementSpeedPID - deltaTime %f, robotDistanceToTarget %f, prevRobotDistanceToTarget %f, propErrorMS %f, intErrorMS %f, derErrorMS %f, movementSpeedPIDPow %f",
                    deltaTimePIDMS, robotDistanceToTarget, prevRobotDistanceToTarget, propErrorMS, intErrorMS, derErrorMS, movementSpeedPIDPow);

        }

        return movementSpeedPIDPow;

    }

    public static double shortParkPID (double robotDistanceToTarget, double prevRobotDistanceToTarget, double deltaTimePIDMS){
        double kPSP = 0.033;
        double kISP = 0.250;
        double kDSP = 0.010;

        //calculate error (Proportional)
        propErrorSP = robotDistanceToTarget;

        //Calculate Total error (Integral)
        intErrorSP += (robotDistanceToTarget) * deltaTimePIDMS / 10;

        //Calculate change in error (Derivative)
        derErrorSP = (robotDistanceToTarget - prevRobotDistanceToTarget) / deltaTimePIDMS;

        double movementSpeedPIDPow = (propErrorSP * kPSP) + (intErrorSP * kISP) + (derErrorSP * kDSP);

        if (debugFlag) {
            RobotLog.d("movementSpeedPID - deltaTime %f, robotDistanceToTarget %f, prevRobotDistanceToTarget %f, propErrorMS %f, intErrorMS %f, derErrorMS %f, movementSpeedPIDPow %f",
                    deltaTimePIDMS, robotDistanceToTarget, prevRobotDistanceToTarget, propErrorMS, intErrorMS, derErrorMS, movementSpeedPIDPow);

        }

        return movementSpeedPIDPow;

    }

    public static double goToPositionParkPID (double robotDistanceToTarget, double prevRobotDistanceToTarget, double deltaTimePIDGTPP){
        double kPMS = 0.033;
        double kIMS = 0.300;
        double kDMS = 0.017;

        //calculate error (Proportional)
        propErrorMS = robotDistanceToTarget;

        //Calculate Total error (Integral)
        intErrorMS += (robotDistanceToTarget) * deltaTimePIDGTPP / 10;

        //Calculate change in error (Derivative)
        derErrorMS = (robotDistanceToTarget - prevRobotDistanceToTarget) / deltaTimePIDGTPP;

        double movementSpeedPIDPow = (propErrorMS * kPMS) + (intErrorMS * kIMS) + (derErrorMS * kDMS);

        if (debugFlag) {
            RobotLog.d("movementSpeedPID - deltaTime %f, robotDistanceToTarget %f, prevRobotDistanceToTarget %f, propErrorMS %f, intErrorMS %f, derErrorMS %f, movementSpeedPIDPow %f",
                    deltaTimePIDGTPP, robotDistanceToTarget, prevRobotDistanceToTarget, propErrorMS, intErrorMS, derErrorMS, movementSpeedPIDPow);

        }

        return movementSpeedPIDPow;

    }


}
