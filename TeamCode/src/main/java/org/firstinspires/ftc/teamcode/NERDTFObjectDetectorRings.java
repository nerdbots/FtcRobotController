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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


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

public class NERDTFObjectDetectorRings extends NERDTFObjectDetector {
    private boolean detected = false;
    public NERDTFObjectDetectorRings(LinearOpMode opmode, String TFOD_MODEL_ASSET, String LABEL_FIRST_ELEMENT, String LABEL_SECOND_ELEMENT, double LABEL_FIRST_ELEMENT_LENGTH, double LABEL_SECOND_ELEMENT_LENGTH, double KNOWN_WIDTH_OF_OBJECT_IN_INCHES) {
       super(opmode, TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT, LABEL_FIRST_ELEMENT_LENGTH, LABEL_SECOND_ELEMENT_LENGTH, KNOWN_WIDTH_OF_OBJECT_IN_INCHES);
   }
    public String identifyRingCase() {

        String ringCase = null;

        if (tfod != null) {
            tfod.activate();
            runtime = new ElapsedTime();
            runtime.reset();
        }

        if (this.opmode.opModeIsActive()) {
            while (this.opmode.opModeIsActive() && !detected && (runtime.seconds() < 1.0)) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    // The TensorFlow software will scale the input images from the camera to a lower resolution.
                    // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                    // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                    // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                    // should be set to the value of the images used to create the TensorFlow Object Detection model
                    // (typically 1.78 or 16/9).

                    // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
                    //tfod.setZoom(2.5, 1.78);
                    if ((updatedRecognitions != null)) {
                        if (updatedRecognitions.size() > 0) {
                            this.opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                            this.opmode.telemetry.update();

                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            //   while (runtime.seconds() <= 5.0 && !detected) {
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(this.LABEL_FIRST_ELEMENT)) {
                                    detected = true;
                                    ringCase= recognition.getLabel();
                                } else if (recognition.getLabel().equals(this.LABEL_SECOND_ELEMENT)) {
                                    detected = true;
                                    ringCase =  recognition.getLabel();
                                }

                            }

                        } else {
                            ringCase =  "None";
                        }
                    }
                }
            }

        }
        tfod.deactivate();
        return ringCase;

    }
    }


