package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="robots got some moves", group="hot dang")



public class robotDance extends LinearOpMode {

    private robot_dance_TeleOp robotDanceTeleOp;




    @Override
    public void runOpMode() {

        robotDanceTeleOp = new robot_dance_TeleOp(this);


        robotDanceTeleOp.initializeHardware();


        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

                robotDanceTeleOp.moveTeleop(0.6, 0, 0.2);

        }


    }
}
