public class CirclePath implements Path {

    double cx, cy, r;
    double startAngleOffset = 0.0;

    public CirclePath(double cx, double cy, double r) {
        this.cx = cx;
        this.cy = cy;
        this.r = r;
    }

    public CirclePath(double cx, double cy, double r, double startAngleOffset) {
        this.cx = cx;
        this.cy = cy;
        this.r = r;
        this.startAngleOffset = startAngleOffset;
    }

    public double[] getPoint(double s) {
        double angle = startAngleOffset + (2 * Math.PI * s);

        double x = cx + r * Math.cos(angle);
        double y = cy + r * Math.sin(angle);

        return new double[]{x, y};
    }

    public void setStartAngleOffset(double offset) {
        this.startAngleOffset = offset;
    }
}
