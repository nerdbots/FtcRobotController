package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import treamcode.CurvePoint;
import treamcode.NerdPID_PurePursuit;
import treamcode.wobble_Pickup;

import java.util.ArrayList;

//import Functions.CurvePoint;

@Autonomous(name="PurePursuitOpMode", group="Linear Opmode")

public class PurePursuitOpMode extends LinearOpMode {

    private PurePursuitRobotMovement6 myPurePursuitRobotMovement6;
    private wobble_Pickup mywobble_Pickup;

    boolean debugFlag = true;

    double purePursuitPath = 0;

    @Override
    public void runOpMode() {
        //Create a NerdBOT object
        myPurePursuitRobotMovement6 = new PurePursuitRobotMovement6(this);
        mywobble_Pickup = new wobble_Pickup (this);

        myPurePursuitRobotMovement6.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6.initializeHardware();
        mywobble_Pickup.wobbleInit();
        //Initialize the PID Calculators

        telemetry.addData("Init", "Completed");
        telemetry.update();

        waitForStart();

        if (purePursuitPath == 4) {
            //First Path to the Square 4
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 40, 0, 0.3));
            allPoints.add(new CurvePoint(-3, 40, 1.0, 0.3, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-11, 113, 1.0, 0.3, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-11, 171, 1.0, 0.3, 40, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 40, 90, 3);

//        Lower arm and release wobble goal
            mywobble_Pickup.beginningDown();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-11, 113, 1.0, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(19, 100, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(21, -44, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 225, 1);

//        Grab wobble goal and raise arm
            mywobble_Pickup.pickupWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 40, 0, 0.3));
            allPoints.add(new CurvePoint(24, 60, 1.0, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-8, 104, 1.0, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-16, 160, 1.0, 0.3, 40, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 90, 3);

//        Lower arm and release wobble goal
            mywobble_Pickup.setDownWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-8, 104, 1.0, 0.4, 40, 0, 0.3));
            allPoints.add(new CurvePoint(4, 60, 1.0, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(4, 10, 1.0, 0.4, 40, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 30, 90, 2);

            mywobble_Pickup.pickupWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(6, 70, 0.6, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(6, 130, 0.6, 0.4, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 7, 90, 4);


//        sleep(1000);

//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(2, 48, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(2, 0, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 3);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(2, 48, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(4, 110, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 2);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(2, 48, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(2, 0, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 3);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(2, 48, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(4, 110, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 2);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(2, 70, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(2, 130, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 7, 90, 4);


        } else if (purePursuitPath == 1) {

        //Second Path to the Square 1
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 35, 0, 0.3));
        allPoints.add(new CurvePoint(-3, 40, 1.0, 0.3, 35, 180, 0.3));
        allPoints.add(new CurvePoint(6, 88, 1.0, 0.3, 35, 180, 0.3));
        allPoints.add(new CurvePoint(27, 148, 1.0, 0.3, 35, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 90, 3);

//        sleep(1000);
        mywobble_Pickup.beginningDown();

        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(6, 88, 1.0, 0.4, 35, 0, 0.3));
        allPoints.add(new CurvePoint(18, 60, 1.0, 0.4, 35, 180, 0.3));
        allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 35, 180, 0.3));
        allPoints.add(new CurvePoint(21, -44, 1.0, 0.3, 35, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 225, 1);

//        sleep(1000);
        mywobble_Pickup.pickupWobble();

        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 35, 0, 0.3));
        allPoints.add(new CurvePoint(18, 60, 1.0, 0.4, 35, 180, 0.3));
        allPoints.add(new CurvePoint(12, 80, 1.0, 0.4, 35, 180, 0.3));
        allPoints.add(new CurvePoint(0, 140, 1.0, 0.3, 35, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 30, 90, 3);

//        sleep(1000);
        mywobble_Pickup.setDownWobble();


        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(12, 86, 0.6, 0.4, 40, 0, 0.3));
        allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 180, 0.3));
        allPoints.add(new CurvePoint(4, 10, 0.6, 0.4, 40, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 3);

        mywobble_Pickup.pickupWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(6, 70, 0.6, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(6, 130, 0.6, 0.4, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 7, 90, 4);


//        sleep(1000);

//        allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 25, 0, 0.3));
//        allPoints.add(new CurvePoint(3, 48, 0.6, 0.4, 25, 180, 0.3));
//        allPoints.add(new CurvePoint(1, 0, 0.6, 0.4, 25, 180, 0.3));
//
//        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 3);
//
//        allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(3, 48, 0.6, 0.4, 40, 0, 0.3));
//        allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 180, 0.3));
//        allPoints.add(new CurvePoint(5, 110, 0.6, 0.4, 40, 180, 0.3));
//
//        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 3);
//
//        allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 0, 0.3));
//        allPoints.add(new CurvePoint(5, 70, 0.6, 0.4, 40, 180, 0.3));
//        allPoints.add(new CurvePoint(5, 130, 0.6, 0.4, 40, 180, 0.3));
//
//        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 7, 90, 4);
        } else {

        //Third Path to the Square 0
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 35, 0, 0.3));
        allPoints.add(new CurvePoint(-12, 66, 1.0, 0.3, 35, 180, 0.3));
        allPoints.add(new CurvePoint(-24, 140, 1.0, 0.3, 35, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 90, 3);

//        sleep(1000);
        mywobble_Pickup.beginningDown();

        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(-18, 66, 1.0, 0.4, 25, 0, 0.3));
        allPoints.add(new CurvePoint(18, 50, 1.0, 0.4, 25, 180, 0.3));
        allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 25, 180, 0.3));
        allPoints.add(new CurvePoint(21, -44, 1.0, 0.3, 25, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 30, 225, 1);

//        sleep(1000);
        mywobble_Pickup.pickupWobble();

        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 35, 0, 0.3));
        allPoints.add(new CurvePoint(-12, 65, 1.0, 0.4, 35, 180, 0.3));
        allPoints.add(new CurvePoint(0, 140, 1.0, 0.3, 35, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 90, 3);

//        sleep(1000);
        mywobble_Pickup.setDownWobble();

        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(-12, 65, 0.6, 0.4, 25, 0, 0.3));
        allPoints.add(new CurvePoint(-12, 50, 0.6, 0.4, 25, 180, 0.3));
        allPoints.add(new CurvePoint(-12, 0, 0.6, 0.4, 25, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 4);

        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(-12, 50, 0.6, 0.4, 25, 0, 0.3));
        allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 25, 180, 0.3));
        allPoints.add(new CurvePoint(24, 100, 0.6, 0.4, 25, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 2);

        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 35, 0, 0.3));
        allPoints.add(new CurvePoint(6, 70, 0.6, 0.4, 35, 180, 0.3));
        allPoints.add(new CurvePoint(6, 130, 0.6, 0.4, 35, 180, 0.3));

        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 7, 90, 4);


        }
    }

}

