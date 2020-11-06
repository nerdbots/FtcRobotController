package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import treamcode.CurvePoint;

import java.util.ArrayList;

//import Functions.CurvePoint;

@Autonomous(name="PurePursuitOpMode", group="Linear Opmode")

public class PurePursuitOpMode extends LinearOpMode {

    private PurePursuitRobotMovement myPurePursuitRobotMovement ;

    boolean debugFlag = true;

    @Override
    public void runOpMode() {
        //Create a NerdBOT object
        myPurePursuitRobotMovement = new PurePursuitRobotMovement(this);

        myPurePursuitRobotMovement.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement.initializeHardware();
        //Initialize the PID Calculators

        telemetry.addData("Init", "Completed");
        telemetry.update();

        waitForStart();

        myPurePursuitRobotMovement.goToPosition(0, 60, 0.6, 90, 0.0);
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
//        myPurePursuitRobotMovement.goToPosition(0, 0, 0.5,0, 0.2);

//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0, 0, 0.8, 0.2, 40, 0, 0.3));
//        allPoints.add(new CurvePoint(45, 30, 0.8, 0.2, 40, 135, 0.3));
//        allPoints.add(new CurvePoint(0, 60, 0.8, 0.2, 40, 135, 0.3));
//        allPoints.add(new CurvePoint(45, 90, 0.8, 0.2, 40, 135, 0.3));
//        allPoints.add(new CurvePoint(0, 120, 0.8, 0.2, 40, 135, 0.3));
//        allPoints.add(new CurvePoint(45, 150, 0.8, 0.2, 40, 135, 0.3));
//
//        myPurePursuitRobotMovement.followCurve(allPoints, 90);

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

