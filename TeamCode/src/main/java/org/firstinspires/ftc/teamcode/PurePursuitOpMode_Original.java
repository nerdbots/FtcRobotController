package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DetectObjects_Shoot_Class;

import treamcode.CurvePoint;
import treamcode.NerdPID_PurePursuit;
import treamcode.wobble_Pickup;

import java.util.ArrayList;

//import Functions.CurvePoint;

@Autonomous(name="Auton (run this!)", group="Auton")

public class PurePursuitOpMode_Original extends LinearOpMode {

    private PurePursuitRobotMovement6 myPurePursuitRobotMovement6;
    //    private wobble_Pickup mywobble_Pickup;
    private DetectObjects_Shoot_Class myDetectObjects;

    boolean debugFlag = true;

    double purePursuitPath;

    @Override
    public void runOpMode() {

//        mywobble_Pickup = new wobble_Pickup (this);
        myDetectObjects = new DetectObjects_Shoot_Class(this);
//        mywobble_Pickup.wobbleInit();
        myDetectObjects.initialize();
        //Initialize the PID Calculators

        purePursuitPath = myDetectObjects.runDetect();

        //  purePursuitPath = 4;

        telemetry.addData("Vision", "Completed DONT PRESS PLAY");
        telemetry.update();

        //Create a NerdBOT object
        myPurePursuitRobotMovement6 = new PurePursuitRobotMovement6(this);
        myPurePursuitRobotMovement6.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6.initializeHardware();

        telemetry.addData("Okay press play", "now");
        telemetry.update();

        waitForStart();

        myPurePursuitRobotMovement6.printI();

//        myPurePursuitRobotMovement6.resetITerm();

        myPurePursuitRobotMovement6.printI();

        myPurePursuitRobotMovement6.resetTimers();

//        myPurePursuitRobotMovement6.runShoot();

        //myDetectObjects.runDetectShoot();

        //    purePursuitPath = 4;

        if (purePursuitPath == 4) {
            //First Path to the Square 4
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 40, 0, 0.3));
            allPoints.add(new CurvePoint(-12, 40, 1.0, 0.3, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-18, 109, 1.0, 0.3, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-18, 171, 1.0, 0.3, 40, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 40, 90, 3);

//        Lower arm and release wobble goal
            myPurePursuitRobotMovement6.beginningDown();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-18, 109, 1.0, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(24, 100, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(24, 23, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(24, -44, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 225, 1);

//        Grab wobble goal and raise arm
            myPurePursuitRobotMovement6.pickupWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 40, 0, 0.3));
            allPoints.add(new CurvePoint(24, 60, 1.0, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-12, 104, 1.0, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-16, 160, 1.0, 0.3, 40, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 90, 3);

//        Lower arm and release wobble goal
            myPurePursuitRobotMovement6.setDownWobble();

            myPurePursuitRobotMovement6.revUpMotor();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-8, 104, 1.0, 0.4, 40, 0, 0.3));
            allPoints.add(new CurvePoint(8, 60, 1.0, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(1, 10, 1.0, 0.4, 40, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 30, 90, 2);

            myPurePursuitRobotMovement6.upWobble();

            //   myPurePursuitRobotMovement6.runShoot();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 35, 0, 0.3));
//            allPoints.add(new CurvePoint(6, 70, 0.6, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(6, 130, 0.6, 0.4, 35, 180, 0.3));
//
//            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 7, 90, 4);


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
            allPoints.add(new CurvePoint(-8, 40, 1.0, 0.3, 35, 180, 0.3));
            allPoints.add(new CurvePoint(6, 88, 1.0, 0.3, 35, 180, 0.3));
            allPoints.add(new CurvePoint(27, 148, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 90, 3);

//        sleep(1000);
            myPurePursuitRobotMovement6.beginningDown();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(6, 88, 1.0, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(20, 60, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(23, 23, 1.0, 0.4, 35, 180, 0.3));
//        allPoints.add(new CurvePoint(21, -44, 1.0, 0.3, 35, 180, 0.3));
            allPoints.add(new CurvePoint(23, -42, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 225, 1);

//        sleep(1000);
            myPurePursuitRobotMovement6.pickupWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(18, 60, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(15, 80, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(0, 140, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 30, 90, 3);

//        sleep(1000);
            myPurePursuitRobotMovement6.setDownWobble();

            myPurePursuitRobotMovement6.revUpMotor();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(12, 86, 0.6, 0.4, 40, 0, 0.3));
            allPoints.add(new CurvePoint(7, 60, 0.6, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(7, 10, 0.6, 0.4, 40, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 3);

            myPurePursuitRobotMovement6.upWobble();

//        myPurePursuitRobotMovement6.runShoot();

//        allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 35, 0, 0.3));
//        allPoints.add(new CurvePoint(6, 70, 0.6, 0.4, 35, 180, 0.3));
//        allPoints.add(new CurvePoint(6, 130, 0.6, 0.4, 35, 180, 0.3));
//
//        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 7, 90, 4);


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
            allPoints.add(new CurvePoint(-18, 60, 1.0, 0.3, 35, 180, 0.3));
            allPoints.add(new CurvePoint(-24, 140, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 90, 3);

//        sleep(1000);
            myPurePursuitRobotMovement6.beginningDown();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-18, 66, 1.0, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(19, 50, 1.0, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(23, 23, 1.0, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(23, -44, 1.0, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 30, 225, 1);

//        sleep(1000);
            myPurePursuitRobotMovement6.pickupWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(21, 23, 1.0, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(-12, 60, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(0, 140, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 35, 90, 3);

//        sleep(1000);
            myPurePursuitRobotMovement6.setDownWobble();

//        allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(-12, 65, 0.6, 0.4, 25, 0, 0.3));
//        allPoints.add(new CurvePoint(-12, 50, 0.6, 0.4, 25, 180, 0.3));
//        allPoints.add(new CurvePoint(-12, 0, 0.6, 0.4, 25, 180, 0.3));
//
//        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 4);

            myPurePursuitRobotMovement6.revUpMotor();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-12, 65, 0.5, 0.4, 10, 0, 0.3));
            allPoints.add(new CurvePoint(-12, 50, 0.5, 0.4, 10, 180, 0.3));
            allPoints.add(new CurvePoint(3, 60, 0.5, 0.4, 10, 180, 0.3));
            allPoints.add(new CurvePoint(33, 80, 0.5, 0.4, 10, 180, 0.3));

            myPurePursuitRobotMovement6.followCurve(allPoints, 90, 10, 90, 2);

            myPurePursuitRobotMovement6.upWobble();

            //  myPurePursuitRobotMovement6.runShoot();

//        allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 35, 0, 0.3));
//        allPoints.add(new CurvePoint(6, 70, 0.6, 0.4, 35, 180, 0.3));
//        allPoints.add(new CurvePoint(6, 130, 0.6, 0.4, 35, 180, 0.3));
//
//        myPurePursuitRobotMovement6.followCurve(allPoints, 90, 7, 90, 4);


        }

        myPurePursuitRobotMovement6.autonEnd(purePursuitPath);


    }

}

