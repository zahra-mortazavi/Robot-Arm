public class CirclePath implements Path{

    double cx,cy,r;

    public CirclePath(double cx,double cy,double r){
        this.cx=cx;
        this.cy=cy;
        this.r=r;
    }

    public double[] getPoint(double s){

        double angle=2*Math.PI*s;

        double x=cx+r*Math.cos(angle);
        double y=cy+r*Math.sin(angle);

        return new double[]{x,y};
    }
}
