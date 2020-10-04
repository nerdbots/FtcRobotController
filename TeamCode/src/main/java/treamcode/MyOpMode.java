package treamcode;

import java.util.ArrayList;

import static treamcode.RobotMovement.followCurve;

public class MyOpMode extends OpMode{

    @Override
    public void init(){

    }

    @Override
    public void loop(){

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0.0,0.0,1.0,1.0,25,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(90,90,1.0,1.0,25,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(110,90,1.0,1.0,25,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(140,50,1.0,1.0,25,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(90,0,1.0,1.0,25,Math.toRadians(50),1.0));

        followCurve(allPoints, Math.toRadians(90));
    }

}
