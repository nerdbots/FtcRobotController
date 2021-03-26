package opencv.core;

//javadoc:Point_
public class PointPP {

    public double x, y;

    public PointPP(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public PointPP() {
        this(0, 0);
    }

    public PointPP(double[] vals) {
        this();
        set(vals);
    }

    public void set(double[] vals) {
        if (vals != null) {
            x = vals.length > 0 ? vals[0] : 0;
            y = vals.length > 1 ? vals[1] : 0;
        } else {
            x = 0;
            y = 0;
        }
    }

    public PointPP clone() {
        return new PointPP(x, y);
    }

    public double dot(PointPP p) {
        return x * p.x + y * p.y;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof PointPP)) return false;
        PointPP it = (PointPP) obj;
        return x == it.x && y == it.y;
    }


    @Override
    public String toString() {
        return "{" + x + ", " + y + "}";
    }
}


