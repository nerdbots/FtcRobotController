package treamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class wobble_Pickup {
    private DcMotor wobbleMotor;
    Servo wobbleServo;

    LinearOpMode opmode;
    private HardwareMap hardwareMap;

    private ElapsedTime Timer = new ElapsedTime();

    public wobble_Pickup(LinearOpMode opmode) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    //Initializes motors (obvi)
    public void wobbleInit() {
        this.wobbleMotor = hardwareMap.get(DcMotor.class, "Left");
        this.wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");
    }

//    Puts the wobble arm down from it's original position. Used at the start of the match.
    public void beginningDown() {
        Timer.reset();
        while(Timer.seconds() < 0.35) {
            wobbleMotor.setPower(0.55);
        }
        wobbleMotor.setPower(0);

        wobbleServo.setPosition(0);

        opmode.sleep(500);

    }

    //Closes servo, then picks up wobble goal
    public void pickupWobble() {
        wobbleServo.setPosition(0.75);

        opmode.sleep(1000);

        Timer.reset();
        while(Timer.seconds() < 0.75) {
            wobbleMotor.setPower(-0.9);
        }
        wobbleMotor.setPower(0.1);

    }
    //Lowers motor, then releases wobble goal
    public void setDownWobble() {
        Timer.reset();
        while(Timer.seconds() < 0.35) {
            wobbleMotor.setPower(0.55);
        }

        wobbleMotor.setPower(0);
        opmode.sleep(500);
        wobbleServo.setPosition(0);
    }

    public void upWobble() {
        wobbleServo.setPosition(0.75);

        opmode.sleep(1000);

        Timer.reset();
        while(Timer.seconds() < 0.75) {
            wobbleMotor.setPower(-0.9);
        }
        wobbleMotor.setPower(0.0);
    }


}
