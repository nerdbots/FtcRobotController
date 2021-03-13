package org.firstinspires.ftc.teamcode;
import com.google.gson.internal.$Gson$Preconditions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

import static android.os.SystemClock.sleep;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;


public class NERDTFObjectDetector {
    protected static final String VUFORIA_KEY =
            "ASaQAYH/////AAABmZRz9hYLmkPytLD5aoLRx68g+hNJkSRzf+Rg0CWPU9iRe1WfGfuNAHReaJ/gzwyVMqKf4VFNTeiMbH6JIYHS2Fzp7aYaCFAU8Zw2zbI8Wa4yV3IP2S7PI9Qdzash8dCuXcaLqU/AHhptdayNn28GsPnsI1YIrYxLlv5i9AkvSokhGZhvABPYtUxWPi7bnQk1JX6ZCNHBz76MhHq1zP24Ce8tIuD/0vWrCt+ZG8cMpxxdRx3Bo85Otq41TIZvjbYM86rsWlGnbeUXJRQfDwYz6zQxMbuY/BGFKZwiXHmmQXaWeIwslAggm+CBA9zGi9x1zbSJEy5MIgVkWJaCCLjx/XOW5z21of9Y+IdIqL0A6qVw";
    protected LinearOpMode opmode;
    protected ElapsedTime runtime;
    protected static HardwareMap hardwareMap;

    protected   final String TFOD_MODEL_ASSET;
    protected   final String LABEL_FIRST_ELEMENT;
    protected   final String LABEL_SECOND_ELEMENT;
    protected  final double LABEL_FIRST_ELEMENT_FOCAL_LENGTH; //In pixels. Use FindFocalLengthCustom.java to find focal length for each object
    protected  final double LABEL_SECOND_ELEMENT_FOCAL_LENGTH; //In pixels. Use FindFocalLengthCustom.java to find focal length for each object
    protected  final double KNOWN_WIDTH_OF_OBJECT; //In inches
    public  static  VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;
    private boolean tfodActivated=false;
    List<Recognition> updatedRecognitions;

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public double findObjectCenter(Recognition recognition) {
        double objectCenter = recognition.getLeft() + recognition.getWidth() / 2.0;
        RobotLog.d("ObjectCenter : %f", objectCenter);
        return objectCenter;
    }

    public double findImageCenter(Recognition recognition) {
        return ((recognition.getImageWidth() / 2.0));
    }


//    public double findDistanceToSingle() {
//        Recognition recognition;
//        recognition = this.detect();
//        return (this.KNOWN_WIDTH_OF_OBJECT * this.LABEL_SECOND_ELEMENT_FOCAL_LENGTH) / recognition.getWidth();
//    }
//
//    public double findDistanceToQuad(Recognition recognition) {
//        return (this.KNOWN_WIDTH_OF_OBJECT * this.LABEL_FIRST_ELEMENT_FOCAL_LENGTH) / recognition.getWidth();
//
//    }

    public double findDistanceToObjectOne(Recognition recognition) {

        return (this.KNOWN_WIDTH_OF_OBJECT * this.LABEL_FIRST_ELEMENT_FOCAL_LENGTH) / recognition.getWidth();

    }

    public double findDistanceToObjectTwo(Recognition recognition) {
        return (this.KNOWN_WIDTH_OF_OBJECT * this.LABEL_SECOND_ELEMENT_FOCAL_LENGTH) / recognition.getWidth();

    }

    public NERDTFObjectDetector(LinearOpMode opmode, String TFOD_MODEL_ASSET, String LABEL_FIRST_ELEMENT, String LABEL_SECOND_ELEMENT, double LABEL_FIRST_ELEMENT_FOCAL_LENGTH, double LABEL_SECOND_ELEMENT_FOCAL_LENGTH, double KNOWN_WIDTH_OF_OBJECT_IN_INCHES) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
        this.TFOD_MODEL_ASSET = TFOD_MODEL_ASSET;
        this.LABEL_FIRST_ELEMENT = LABEL_FIRST_ELEMENT;
        this.LABEL_SECOND_ELEMENT = LABEL_SECOND_ELEMENT;
        this.LABEL_FIRST_ELEMENT_FOCAL_LENGTH = LABEL_FIRST_ELEMENT_FOCAL_LENGTH;
        this.LABEL_SECOND_ELEMENT_FOCAL_LENGTH = LABEL_SECOND_ELEMENT_FOCAL_LENGTH;
        this.KNOWN_WIDTH_OF_OBJECT = KNOWN_WIDTH_OF_OBJECT_IN_INCHES;
    }
    public NERDTFObjectDetector(LinearOpMode opmode, String TFOD_MODEL_ASSET, String LABEL_FIRST_ELEMENT, double LABEL_FIRST_ELEMENT_FOCAL_LENGTH,  double KNOWN_WIDTH_OF_OBJECT_IN_INCHES) {
        this(opmode, TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, "", LABEL_FIRST_ELEMENT_FOCAL_LENGTH, 0, 8);
    }



    public  static void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    public void initialize() {
        initVuforia();
        initTfod();
    }
    public Recognition detect(String labelToDetect, boolean continuousDetection) {
        Recognition detectedRecognition = null;
        if (tfod != null) {
            if(!tfodActivated){
                tfod.activate();
                tfodActivated=true;
            }
            runtime = new ElapsedTime();
            runtime.reset();

        }

        if (this.opmode.opModeIsActive()) {
            while (this.opmode.opModeIsActive()  && (runtime.seconds() < 1.0)) {

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
                                if(recognition.getLabel().equals(labelToDetect)){
                                    detectedRecognition = recognition;
                                    break;
                                }

                            }

                        }
                    }
                }

            }
        }
        if(!continuousDetection)tfod.deactivate();

        return detectedRecognition;
    }

}















































//        ░░░░░░░░░░░░░░░░██████████████████
//        ░░░░░░░░░░░░████░░░░░░░░░░░░░░░░░░████
//        ░░░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░██
//        ░░░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░██
//        ░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░██
//        ░░░░░░░░██░░░░░░░░░░░░░░░░░░░░██████░░░░██
//        ░░░░░░░░██░░░░░░░░░░░░░░░░░░░░██████░░░░██
//        ░░░░░░░░██░░░░██████░░░░██░░░░██████░░░░██
//        ░░░░░░░░░░██░░░░░░░░░░██████░░░░░░░░░░██
//        ░░░░░░░░████░░██░░░░░░░░░░░░░░░░░░██░░████
//        ░░░░░░░░██░░░░██████████████████████░░░░██
//        ░░░░░░░░██░░░░░░██░░██░░██░░██░░██░░░░░░██
//        ░░░░░░░░░░████░░░░██████████████░░░░████
//        ░░░░░░░░██████████░░░░░░░░░░░░░░██████████
//        ░░░░░░██░░██████████████████████████████░░██
//        ░░░░████░░██░░░░██░░░░░░██░░░░░░██░░░░██░░████
//        ░░░░██░░░░░░██░░░░██████░░██████░░░░██░░░░░░██
//        ░░██░░░░████░░██████░░░░██░░░░██████░░████░░░░██
//        ░░██░░░░░░░░██░░░░██░░░░░░░░░░██░░░░██░░░░░░░░██
//        ░░██░░░░░░░░░░██░░██░░░░░░░░░░██░░██░░░░░░░░░░██
//        ░░░░██░░░░░░██░░░░████░░░░░░████░░░░██░░░░░░██
//        ░░░░░░████░░██░░░░██░░░░░░░░░░██░░░░██░░████
//        ░░░░░░░░██████░░░░██████████████░░░░██████
//        ░░░░░░░░░░████░░░░██████████████░░░░████
//        ░░░░░░░░██████████████████████████████████
//        ░░░░░░░░████████████████░░████████████████
//        ░░░░░░░░░░████████████░░░░░░████████████
//        ░░░░░░██████░░░░░░░░██░░░░░░██░░░░░░░░██████
//        ░░░░░░██░░░░░░░░░░████░░░░░░████░░░░░░░░░░██
//        ░░░░░░░░██████████░░░░░░░░░░░░░░██████████