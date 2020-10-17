package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import treamcode.CurvePoint;
import treamcode.NerdPIDCalc_MotionProfiling_Clean;

import java.util.ArrayList;

//import Functions.CurvePoint;

@Autonomous(name="PurePursuitOpMode", group="Linear Opmode")

public class PurePursuitOpMode extends LinearOpMode {

    private PurePursuitRobotMovement myPurePursuitRobotMovement ;

    private NerdPIDCalc_MotionProfiling_Clean VPID;

    boolean debugFlag = true;

    @Override
    public void runOpMode() {
        VPID = new NerdPIDCalc_MotionProfiling_Clean(this);

        //Create a NerdBOT object
        myPurePursuitRobotMovement = new PurePursuitRobotMovement(this);

        myPurePursuitRobotMovement.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement.initializeHardware();
        //Initialize the PID Calculators
        double aP = 0.012; //0.009;, 0.025; 2.005
        double aI = 0.0022; //0.12;, 0.08
        double aD = 0.0;

        VPID.setFLGains(aP, aI, aD);
        VPID.setFRGains(aP, aI, aD);
        VPID.setRLGains(aP, aI, aD);
        VPID.setRRGains(aP, aI, aD);//.00732
        VPID.setGyroGains(0.5,     0.5, 0);//0.015

        VPID.initializeHardware();

        sleep(700);


        telemetry.addData("Init", "Completed");
        telemetry.update();

        waitForStart();

 //       myPurePursuitRobotMovement.goToPosition(0, 60, 0.3, 90, 0.0);
//        myPurePursuitRobotMovement.goToPosition(-90,45, 0.8, 135, 0.2);
//        myPurePursuitRobotMovement.goToPosition(0,0,0.8,135,0.2);

//        myPurePursuitRobotMovement.goToPosition(40, 80, 0.5, 90, 0.2);
//        myPurePursuitRobotMovement.goToPosition(-40, 80, 0.5, 0, 0.2);
//        myPurePursuitRobotMovement.goToPosition(-40, 140, 0.5, 0, 0.2);
//        myPurePursuitRobotMovement.goToPosition(45, 140, 0.5, 0, 0.2);
//        myPurePursuitRobotMovement.goToPosition(45, 45, 0.5, 0, 0.2);
//        myPurePursuitRobotMovement.goToPosition(0, 0, 0.5,0, 0.2);

//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0, 0, 0.8, 0.2, 25, 0, 0.3));
//        allPoints.add(new CurvePoint(30, 45, 0.8, 0.2, 25, 135, 0.3));
//        allPoints.add(new CurvePoint(0, 90, 0.8, 0.2, 25, 135, 0.3));
//        allPoints.add(new CurvePoint(30, 135, 0.8, 0.2, 25, 135, 0.3));
//
//        myPurePursuitRobotMovement.followCurve(allPoints, 90);

//        myPurePursuitRobotMovement.goToPosition(45, 30, 0.8, 135, 0.2);
//        myPurePursuitRobotMovement.goToPosition(0, 60, 0.8, 90, 0.2);
//        myPurePursuitRobotMovement.goToPosition(45, 90, 0.8, 135, 0.2);
//        myPurePursuitRobotMovement.goToPosition(0, 120, 0.8, 90, 0.2);
//        myPurePursuitRobotMovement.goToPosition(45, 150, 0.8, 135, 0.2);
        //myPurePursuitRobotMovement.goToPosition(0, 0, 0.5,0, 0.2);

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 0.8, 0.2, 40, 0, 0.3));
        allPoints.add(new CurvePoint(45, 30, 0.8, 0.2, 40, 135, 0.3));
        allPoints.add(new CurvePoint(0, 60, 0.8, 0.2, 40, 135, 0.3));
        allPoints.add(new CurvePoint(45, 90, 0.8, 0.2, 40, 135, 0.3));
        allPoints.add(new CurvePoint(0, 120, 0.8, 0.2, 40, 135, 0.3));
        allPoints.add(new CurvePoint(45, 150, 0.8, 0.2, 40, 135, 0.3));

        myPurePursuitRobotMovement.followCurve(allPoints, 90);

//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0, 0, 0.8, 0.2, 25, 0, 0.3));
//        allPoints.add(new CurvePoint(45, 45, 0.8, 0.2, 25, 135, 0.3));
//        allPoints.add(new CurvePoint(45, 60, 0.8, 0.2, 25, 135, 0.3));
//        allPoints.add(new CurvePoint(0 , 80, 0.8, 0.2, 25, 135, 0.3));
//        allPoints.add(new CurvePoint(-45, 80, 0.8, 0.2, 25, 135, 0.3));
//
//        myPurePursuitRobotMovement.followCurve(allPoints, 90);

//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0, 0, 0.5, 0.2, 25, 0, 0.0));
//        allPoints.add(new CurvePoint(45, 45, 0.5, 0.2, 25, 0, 0.0));
//        allPoints.add(new CurvePoint(-90, 45, 0.5, 0.2, 25, 0, 0.0));
//        allPoints.add(new CurvePoint(-90, 0, 0.5, 0.2, 25, 0, 0.3));
//        allPoints.add(new CurvePoint(-90, -25, 0.5, 0.2, 25, 0, 0.3));
//        //allPoints.add(new CurvePoint(-45, -25, 0.5, 0.2, 25, 135, 0.3));
//
//        myPurePursuitRobotMovement.followCurve(allPoints, 90);


    }

}

