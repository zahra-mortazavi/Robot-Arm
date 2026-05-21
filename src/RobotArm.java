public class RobotArm {

    public double A1, A2, A3;

    public double theta1 = 0.5;
    public double theta2 = -0.6;
    public double theta3 = 0.3;

    public double thetaDot1 = 0;
    public double thetaDot2 = 0;
    public double thetaDot3 = 0;

    public double j1_min, j1_max;
    public double j2_min, j2_max;
    public double j3_min, j3_max;


    public RobotArm(double a1, double a2, double a3){

        A1 = a1;
        A2 = a2;
        A3 = a3;

        j1_min = Math.toRadians(-180);
        j1_max = Math.toRadians(180);

        j2_min = Math.toRadians(-180);
        j2_max = Math.toRadians(180);

        j3_min = Math.toRadians(-180);
        j3_max = Math.toRadians(180);
    }

    public RobotArm(double A1,double A2,double A3,
                    double j1Min,double j1Max,
                    double j2Min,double j2Max,
                    double j3Min,double j3Max){

        this.A1 = A1;
        this.A2 = A2;
        this.A3 = A3;

        this.j1_min = j1Min;
        this.j1_max = j1Max;

        this.j2_min = j2Min;
        this.j2_max = j2Max;

        this.j3_min = j3Min;
        this.j3_max = j3Max;
    }

    private double clamp(double v,double min,double max){
        return Math.max(min, Math.min(max,v));
    }

    private void enforceLimits(){

        theta1 = clamp(theta1,j1_min,j1_max);
        theta2 = clamp(theta2,j2_min,j2_max);
        theta3 = clamp(theta3,j3_min,j3_max);
    }


    public double[] forwardKinematics(){

        double x1 = A1*Math.cos(theta1);
        double y1 = A1*Math.sin(theta1);

        double x2 = x1 + A2*Math.cos(theta1+theta2);
        double y2 = y1 + A2*Math.sin(theta1+theta2);

        double x3 = x2 + A3*Math.cos(theta1+theta2+theta3);
        double y3 = y2 + A3*Math.sin(theta1+theta2+theta3);

        return new double[]{0,0,x1,y1,x2,y2,x3,y3};
    }


    public double[] endEffectorVelocity(){

        double s1 = Math.sin(theta1);
        double s12 = Math.sin(theta1+theta2);
        double s123 = Math.sin(theta1+theta2+theta3);

        double c1 = Math.cos(theta1);
        double c12 = Math.cos(theta1+theta2);
        double c123 = Math.cos(theta1+theta2+theta3);

        double j11 = -A1*s1 - A2*s12 - A3*s123;
        double j12 = -A2*s12 - A3*s123;
        double j13 = -A3*s123;

        double j21 = A1*c1 + A2*c12 + A3*c123;
        double j22 = A2*c12 + A3*c123;
        double j23 = A3*c123;

        double vx = j11*thetaDot1 + j12*thetaDot2 + j13*thetaDot3;
        double vy = j21*thetaDot1 + j22*thetaDot2 + j23*thetaDot3;

        return new double[]{vx,vy};
    }


    public void inverseKinematics(double xe,double ye){

        double phi = 0; // default orientation

        inverseKinematics(xe,ye,phi);
    }


    public void inverseKinematics(double xe,double ye,double phi){

        double xw = xe - A3*Math.cos(phi);
        double yw = ye - A3*Math.sin(phi);

        double r2 = xw*xw + yw*yw;
        double r = Math.sqrt(r2);

        double cosB = (A1*A1 + A2*A2 - r2)/(2*A1*A2);
        cosB = clamp(cosB,-1,1);

        double beta = Math.acos(cosB);
        double newTheta2 = Math.PI - beta;

        double alpha = Math.atan2(yw,xw);

        double cosG = (r2 + A1*A1 - A2*A2)/(2*r*A1);
        cosG = clamp(cosG,-1,1);

        double gamma = Math.acos(cosG);

        double newTheta1 = alpha - gamma;
        double newTheta3 = phi - newTheta1 - newTheta2;

        thetaDot1 = newTheta1 - theta1;
        thetaDot2 = newTheta2 - theta2;
        thetaDot3 = newTheta3 - theta3;

        theta1 = newTheta1;
        theta2 = newTheta2;
        theta3 = newTheta3;

        enforceLimits();
    }


    public double[] computeIKRaw(double xe,double ye,double phi) throws Exception{

        double xw = xe - A3*Math.cos(phi);
        double yw = ye - A3*Math.sin(phi);

        double r2 = xw*xw + yw*yw;
        double r = Math.sqrt(r2);

        double cosB = (A1*A1 + A2*A2 - r2)/(2*A1*A2);
        if(cosB<-1 || cosB>1) throw new Exception("No IK");

        double beta = Math.acos(cosB);
        double T2 = Math.PI - beta;

        double alpha = Math.atan2(yw,xw);

        double cosG = (r2 + A1*A1 - A2*A2)/(2*r*A1);
        if(cosG<-1 || cosG>1) throw new Exception("No IK");

        double gamma = Math.acos(cosG);

        double T1 = alpha - gamma;
        double T3 = phi - T1 - T2;

        return new double[]{T1,T2,T3};
    }


    public void manualTarget(double xe,double ye,double phi){

        try{

            double[] angles = computeIKRaw(xe,ye,phi);

            theta1 = angles[0];
            theta2 = angles[1];
            theta3 = angles[2];

            enforceLimits();

        }catch(Exception e){

            System.out.println("Target out of reach");
        }
    }


    public void jacobianControlStep(double xd,double yd,double dt){

        double[] p = forwardKinematics();

        double xe = p[6];
        double ye = p[7];

        double k = 5;

        double vx = k*(xd-xe);
        double vy = k*(yd-ye);

        double s1 = Math.sin(theta1);
        double s12 = Math.sin(theta1+theta2);
        double s123 = Math.sin(theta1+theta2+theta3);

        double c1 = Math.cos(theta1);
        double c12 = Math.cos(theta1+theta2);
        double c123 = Math.cos(theta1+theta2+theta3);

        double J11 = -A1*s1 - A2*s12 - A3*s123;
        double J12 = -A2*s12 - A3*s123;
        double J13 = -A3*s123;

        double J21 = A1*c1 + A2*c12 + A3*c123;
        double J22 = A2*c12 + A3*c123;
        double J23 = A3*c123;

        double a = J11*J11 + J12*J12 + J13*J13;
        double b = J11*J21 + J12*J22 + J13*J23;
        double c = J21*J21 + J22*J22 + J23*J23;

        double det = a*c - b*b;

        if(Math.abs(det) < 1e-6) return;

        double inv11 = c/det;
        double inv12 = -b/det;
        double inv21 = -b/det;
        double inv22 = a/det;

        double t1 = J11*inv11 + J21*inv21;
        double t2 = J11*inv12 + J21*inv22;

        double t3 = J12*inv11 + J22*inv21;
        double t4 = J12*inv12 + J22*inv22;

        double t5 = J13*inv11 + J23*inv21;
        double t6 = J13*inv12 + J23*inv22;

        double theta1_dot = t1*vx + t2*vy;
        double theta2_dot = t3*vx + t4*vy;
        double theta3_dot = t5*vx + t6*vy;

        thetaDot1 = theta1_dot;
        thetaDot2 = theta2_dot;
        thetaDot3 = theta3_dot;

        theta1 += theta1_dot*dt;
        theta2 += theta2_dot*dt;
        theta3 += theta3_dot*dt;

        enforceLimits();
    }
    public double[] joint1Velocity(){

        double vx = -A1*Math.sin(theta1)*thetaDot1;
        double vy =  A1*Math.cos(theta1)*thetaDot1;

        return new double[]{vx,vy};
    }

    public double[] joint2Velocity(){

        double vx =
                -A1*Math.sin(theta1)*thetaDot1
                        -A2*Math.sin(theta1+theta2)*(thetaDot1+thetaDot2);

        double vy =
                A1*Math.cos(theta1)*thetaDot1
                        +A2*Math.cos(theta1+theta2)*(thetaDot1+thetaDot2);

        return new double[]{vx,vy};
    }

    public double[] joint3Velocity(){

        double vx =
                -A1*Math.sin(theta1)*thetaDot1
                        -A2*Math.sin(theta1+theta2)*(thetaDot1+thetaDot2)
                        -A3*Math.sin(theta1+theta2+theta3)*(thetaDot1+thetaDot2+thetaDot3);

        double vy =
                A1*Math.cos(theta1)*thetaDot1
                        +A2*Math.cos(theta1+theta2)*(thetaDot1+thetaDot2)
                        +A3*Math.cos(theta1+theta2+theta3)*(thetaDot1+thetaDot2+thetaDot3);

        return new double[]{vx,vy};
    }
    public double[] endEffectorPosition(){

        double x =
                A1*Math.cos(theta1)
                        + A2*Math.cos(theta1 + theta2)
                        + A3*Math.cos(theta1 + theta2 + theta3);

        double y =
                A1*Math.sin(theta1)
                        + A2*Math.sin(theta1 + theta2)
                        + A3*Math.sin(theta1 + theta2 + theta3);

        return new double[]{x,y};
    }

}
