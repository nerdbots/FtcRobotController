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

    public void wobble_Pickup(LinearOpMode opmode) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    //Initializes motors (obvi)
    public void wobbleInit() {
        this.wobbleMotor = hardwareMap.get(DcMotor.class, "Left");
        this.wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");
    }

    //Puts the wobble arm down from it's original position. Used at the start of the match.
    public void beginningDown() {
            Timer.reset();
            while(Timer.seconds() < 0.25) {
                this.wobbleMotor.setPower(-0.55);
            }
            this.wobbleMotor.setPower(0);

            this.wobbleServo.setPosition(0);

    }

    //Closes servo, then picks up wobble goal
    public void pickupWobble() {
        this.wobbleServo.setPosition(1);

        opmode.sleep(500);

        Timer.reset();
        while(Timer.seconds() < 0.53) {
            this.wobbleMotor.setPower(0.9);
        }
        this.wobbleMotor.setPower(0);

    }
    //Lowers motor, then releases wobble goal
    public void setDownWobble() {
        Timer.reset();
        while(Timer.seconds() < 0.17) {
            wobbleMotor.setPower(-0.55);
        }

        wobbleMotor.setPower(0);
        opmode.sleep(500);
        wobbleServo.setPosition(0);
    }


}
