 /* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static android.os.SystemClock.sleep;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Disabled

public class NERDShooterClass_TeleOp{
    //Set these two variables to the indexer home position and the indexer pushed position
    public static final double indexerHomePos = 1;
    public static final double indexerPushedPos = 0.2;
    private HardwareMap hardwareMap;
    public DcMotorEx shooterMotor;
    public Servo indexingServo;
    public DcMotor intakeMotor;
    private Servo kickerServo;

    private int shooterveloc = -1425;
    private boolean shouldIndex = false;
    LinearOpMode opMode;
    public NERDShooterClass_TeleOp(LinearOpMode opmode) {
        this.opMode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    //Function to index rings. This will not run the motor. Run the motor function first,
    //before using this function to index the rings
    public void indexRings() {
        int count = 0;
        boolean cycle = true;
        indexingServo.setPosition(indexerHomePos);
        while(count < 8){
            double position = 0.0;


            if(shouldIndex) {
                if (cycle) {
                    //Make Indexer go forward
                    position = this.indexerPushedPos;
                    cycle = false;
                } else {
                    //Make Indexer go backward
                    position = this.indexerHomePos;
                    cycle = true;
                }

                if (count == 6 || count == 7) {
                    intakeMotor.setPower(-1);
                }
            }

            // Set the servo to the new position and pause;
            indexingServo.setPosition(position);

            if((Math.abs(Math.abs(shooterMotor.getVelocity()) - Math.abs(shooterveloc)) < 20)) {
                shouldIndex = true;
                count++;
            } else {
                shouldIndex = false;
            }
        }
        //Stop Motor
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
    }
    //Function to run shooter motor. Run this function and then sleep for a little bit to let
    //the motor charge up. Then run the indexRings function
    public void runShooterMotor(double motorPower) {
        shooterMotor.setPower(motorPower);
    }

    public void stopShooterMotor() {
        runShooterMotor(0);
    }

    //Initialize the shooter and indexer
    public void initialize() {
        //Change the deviceName parameter to whatever the motor/servo is named in your config
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Front");
        indexingServo = hardwareMap.get(Servo.class, "indexingServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "Right");
        kickerServo = hardwareMap.get(Servo.class, "Kicker_Servo");
        indexingServo.setPosition(indexerHomePos);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void indexRingsOnce() {
        boolean cycle = true;
        kickerServo.setPosition(-1);
        indexingServo.setPosition(indexerHomePos);
        double position = indexerHomePos;
        double position2 = 0;
        int count = 0;
        while(count < 2) {
            if (cycle) {
                //Make Indexer go forward
                position = this.indexerPushedPos;
                position2 = 1;
                cycle = false;
            } else {
                //Make Indexer go backward
                position = this.indexerHomePos;
                position2 = -1;
                cycle = true;
            }
            indexingServo.setPosition(position);
            indexingServo.setPosition(position2);
            sleep(400);
            count++;
        }
    }
}

