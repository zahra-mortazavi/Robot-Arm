public class LinePath implements Path{

    double x1,y1,x2,y2;

    public LinePath(double x1,double y1,double x2,double y2){
        this.x1=x1;
        this.y1=y1;
        this.x2=x2;
        this.y2=y2;
    }

    public double[] getPoint(double s){

        double x=x1+(x2-x1)*s;
        double y=y1+(y2-y1)*s;

        return new double[]{x,y};
    }
}
