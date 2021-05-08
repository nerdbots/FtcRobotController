package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import treamcode.CurvePoint;
import treamcode.wobble_Pickup;

//import Functions.CurvePoint;

@Autonomous(name="park test", group="Linear Opmode")

public class backupParkTest extends LinearOpMode {

    private PurePursuitRobotMovement6 myPurePursuitRobotMovement6;
    private wobble_Pickup mywobble_Pickup;
    private DetectObjects_Shoot_Class myDetectObjects;
  //  private NERDShooterClass nerdShooterClass;

    private int ringsNum;

    boolean debugFlag = true;

    double purePursuitPath;

    @Override
    public void runOpMode() {
        //Create a NerdBOT object
        myPurePursuitRobotMovement6 = new PurePursuitRobotMovement6(this);
        mywobble_Pickup = new wobble_Pickup (this);
        myDetectObjects = new DetectObjects_Shoot_Class(this);
       // nerdShooterClass = new NERDShooterClass(this);

        myPurePursuitRobotMovement6.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6.initializeHardware();
        mywobble_Pickup.wobbleInit();
      //  nerdShooterClass.initialize();
        myDetectObjects.initialize();

        //Initialize the PID Calculators

     //   purePursuitPath = myDetectObjects.runDetect();

        purePursuitPath = 4;
        telemetry.addData("Init", "DID A THING");
        telemetry.update();

        waitForStart();

        myPurePursuitRobotMovement6.revUpMotor();

       myPurePursuitRobotMovement6.autonEnd(purePursuitPath);

       // myPurePursuitRobotMovement6.resetBackEncoder();



    }

}
