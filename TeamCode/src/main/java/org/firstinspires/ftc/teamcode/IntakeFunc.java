package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class IntakeFunc {
    protected LinearOpMode opMode;
    protected ElapsedTime runtime;
    protected static HardwareMap hardwareMap;
    protected DcMotor intake;
    protected double intakePower = 0;
    protected Servo servo;
    protected DcMotor motor;
    protected double  position = 0; // Start at halfway position
    protected boolean rampUp = true;
    protected boolean cycle = true;
    protected int count = 0;


    public void initIntake(){
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void initShooter(){
        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Starting Pos for servo
        servo.setPosition(0.3);
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
    }

    public void collectRings(){
        intake.setPower(intakePower);
    }

    public void shoot() throws InterruptedException {
        int count = 0;
        //Set count higher based on how many times you want the indexer to move back and forth
        while(count < 5){
            //Setting the speed of the shooter wheel
            motor.setPower(-1);

            if(count == 0) {
                //Wait for the Shooter wheel to reach full speed
                sleep(5000);
            }
            if (cycle) {
                //Make Indexer go forward
                position = 0.0;
                cycle = false;
            } else {
                //Make Indexer go backward
                position = 0.3;
                cycle = true;
            }

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            sleep(500);
            count++;
        }
        //Stop Motor
        motor.setPower(0);
    }

}
