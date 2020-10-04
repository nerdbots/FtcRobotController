package treamcode;

import opencv.core.Point;

import java.util.ArrayList;

public class MathFunctions {

    public static double AngleWrap(double angle) {
        while (angle < Math.PI) {
            angle += 2 * Math.PI;

        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle;

    }

    public static double AngleWrapDeg(double angle){
        while (angle < -180){
            angle += 360;
        }
        while (angle > 180){
            angle -= 360;
        }

        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection (Point circleCenter, double radius,
                                                           Point linePoint1, Point linePoint2){

        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.007;


        }

        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.007;
        }


        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);


        double quadraticA = 1.0 + Math.pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;



        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);

        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1, 2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try{

            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2 * quadraticA);

            double yRoot1 = m1 *  (xRoot1 - x1) + y1;

            //" put back the offset. Who says that?! put back the offset... K, I don't care." - Gluten Free, FTC team 11115
            xRoot1 += circleCenter.x;
            //>>>>>>>>>>>>>>>>>>>>>>>> do later, define yRoot1 <<<<<<<<<<<<<<<<<<<<<<<<
            yRoot1 += circleCenter.y;

            //BOUNDING BOX!!!!!!1!
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
            double minY = linePoint1.y < linePoint2.y ? linePoint1.y : linePoint2.y;
            double maxY = linePoint1.y > linePoint2.y ? linePoint1.y : linePoint2.y;
            //BOUNDING BOX!!!!!!11!!!

            if(xRoot1 > minX && xRoot1 < maxX && yRoot1 > minY && yRoot1 < maxY){
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2 * quadraticA);
            double yRoot2 = m1 *  (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;


            if(xRoot2 > minX && xRoot2 < maxX && yRoot2 > minY && yRoot2 < maxY){
                allPoints.add(new Point(xRoot2, yRoot2));

            }


        }catch (Exception e){

        }
        return allPoints;



    }
}
