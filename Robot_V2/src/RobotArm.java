import java.awt.*;

public class RobotArm {

    public double A1, A2, A3;

    public double theta1 = 0.5;
    public double theta2 = -0.6;
    public double theta3 = 0.3;

    public double thetaDot1 = 0;
    public double thetaDot2 = 0;
    public double thetaDot3 = 0;

    public double j1Min, j1Max;
    public double j2Min, j2Max;
    public double j3Min, j3Max;

    public RobotArm(double A1, double A2, double A3,
                    double j1Min, double j1Max,
                    double j2Min, double j2Max,
                    double j3Min, double j3Max) {

        this.A1 = A1;
        this.A2 = A2;
        this.A3 = A3;

        this.j1Min = j1Min;
        this.j1Max = j1Max;

        this.j2Min = j2Min;
        this.j2Max = j2Max;

        this.j3Min = j3Min;
        this.j3Max = j3Max;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public void enforceLimits() {
        theta1 = clamp(theta1, j1Min, j1Max);
        theta2 = clamp(theta2, j2Min, j2Max);
        theta3 = clamp(theta3, j3Min, j3Max);
    }

    private double[] computeAllPositions() {
        double x1 = A1 * Math.cos(theta1);
        double y1 = A1 * Math.sin(theta1);

        double x2 = x1 + A2 * Math.cos(theta1 + theta2);
        double y2 = y1 + A2 * Math.sin(theta1 + theta2);

        double x3 = x2 + A3 * Math.cos(theta1 + theta2 + theta3);
        double y3 = y2 + A3 * Math.sin(theta1 + theta2 + theta3);

        return new double[]{x1, y1, x2, y2, x3, y3};
    }

    public double[] forwardKinematics() {
        double[] pos = computeAllPositions();
        return new double[]{0, 0, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]};
    }

    private double[] computeJacobian() {
        double s1 = Math.sin(theta1);
        double s12 = Math.sin(theta1 + theta2);
        double s123 = Math.sin(theta1 + theta2 + theta3);

        double c1 = Math.cos(theta1);
        double c12 = Math.cos(theta1 + theta2);
        double c123 = Math.cos(theta1 + theta2 + theta3);

        double J11 = -A1 * s1 - A2 * s12 - A3 * s123;
        double J12 = -A2 * s12 - A3 * s123;
        double J13 = -A3 * s123;

        double J21 = A1 * c1 + A2 * c12 + A3 * c123;
        double J22 = A2 * c12 + A3 * c123;
        double J23 = A3 * c123;

        return new double[]{J11, J12, J13, J21, J22, J23};
    }

    public double[] endEffectorVelocity() {
        double[] J = computeJacobian();
        double vx = J[0] * thetaDot1 + J[1] * thetaDot2 + J[2] * thetaDot3;
        double vy = J[3] * thetaDot1 + J[4] * thetaDot2 + J[5] * thetaDot3;
        return new double[]{vx, vy};
    }

    private double[][] computeAllJointVelocities() {
        double s1 = Math.sin(theta1);
        double s12 = Math.sin(theta1 + theta2);
        double s123 = Math.sin(theta1 + theta2 + theta3);

        double c1 = Math.cos(theta1);
        double c12 = Math.cos(theta1 + theta2);
        double c123 = Math.cos(theta1 + theta2 + theta3);

        double vx1 = -A1 * s1 * thetaDot1;
        double vy1 = A1 * c1 * thetaDot1;

        double vx2 = vx1 - A2 * s12 * (thetaDot1 + thetaDot2);
        double vy2 = vy1 + A2 * c12 * (thetaDot1 + thetaDot2);

        double vx3 = vx2 - A3 * s123 * (thetaDot1 + thetaDot2 + thetaDot3);
        double vy3 = vy2 + A3 * c123 * (thetaDot1 + thetaDot2 + thetaDot3);

        return new double[][]{{vx1, vy1}, {vx2, vy2}, {vx3, vy3}};
    }

    public double[] joint1Velocity() {
        return computeAllJointVelocities()[0];
    }

    public double[] joint2Velocity() {
        return computeAllJointVelocities()[1];
    }

    public double[] joint3Velocity() {
        return computeAllJointVelocities()[2];
    }

    //   سینماتیک معکوس (با امکان clamp یا throw)
    private double[] computeIKCore(double xe, double ye, double phi, boolean clampValues) throws Exception {
        double xw = xe - A3 * Math.cos(phi);
        double yw = ye - A3 * Math.sin(phi);

        double r2 = xw * xw + yw * yw;
        double r = Math.sqrt(r2);

        double cosB = (A1 * A1 + A2 * A2 - r2) / (2 * A1 * A2);
        if (clampValues) {
            cosB = clamp(cosB, -1, 1);
        } else if (cosB < -1 || cosB > 1) {
            throw new Exception("No IK");
        }

        double beta = Math.acos(cosB);
        double newTheta2 = Math.PI - beta;

        double alpha = Math.atan2(yw, xw);

        double cosG = (r2 + A1 * A1 - A2 * A2) / (2 * r * A1);
        if (clampValues) {
            cosG = clamp(cosG, -1, 1);
        } else if (cosG < -1 || cosG > 1) {
            throw new Exception("No IK");
        }

        double gamma = Math.acos(cosG);

        double newTheta1 = alpha - gamma;
        double newTheta3 = phi - newTheta1 - newTheta2;

        return new double[]{newTheta1, newTheta2, newTheta3};
    }

    public double[] computeIKRaw(double xe, double ye, double phi) throws Exception {
        return computeIKCore(xe, ye, phi, false);
    }


    public void inverseKinematics(double xe, double ye, double phi) throws Exception {
        double[] angles = computeIKCore(xe, ye, phi, true);
        double newTheta1 = angles[0];
        double newTheta2 = angles[1];
        double newTheta3 = angles[2];

        thetaDot1 = newTheta1 - theta1;
        thetaDot2 = newTheta2 - theta2;
        thetaDot3 = newTheta3 - theta3;

        theta1 = newTheta1;
        theta2 = newTheta2;
        theta3 = newTheta3;

        enforceLimits();
    }

    public void manualTarget(double xe, double ye, double phi) {
        try {
            double[] angles = computeIKRaw(xe, ye, phi);
            theta1 = angles[0];
            theta2 = angles[1];
            theta3 = angles[2];
            enforceLimits();
        } catch (Exception e) {
            System.out.println("Target out of reach");
        }
    }

    public void jacobianControlStep(double xd, double yd, double dt) {
        double[] endPos = endEffectorPosition();
        double xe = endPos[0];
        double ye = endPos[1];

        double k = 5;
        double vx = k * (xd - xe);
        double vy = k * (yd - ye);

        double[] J = computeJacobian();
        double J11 = J[0], J12 = J[1], J13 = J[2];
        double J21 = J[3], J22 = J[4], J23 = J[5];

        double a = J11 * J11 + J12 * J12 + J13 * J13;
        double b = J11 * J21 + J12 * J22 + J13 * J23;
        double c = J21 * J21 + J22 * J22 + J23 * J23;

        double det = a * c - b * b;
        if (Math.abs(det) < 1e-6) return;

        double inv11 = c / det;
        double inv12 = -b / det;
        double inv21 = -b / det;
        double inv22 = a / det;

        double t1 = J11 * inv11 + J21 * inv21;
        double t2 = J11 * inv12 + J21 * inv22;
        double t3 = J12 * inv11 + J22 * inv21;
        double t4 = J12 * inv12 + J22 * inv22;
        double t5 = J13 * inv11 + J23 * inv21;
        double t6 = J13 * inv12 + J23 * inv22;

        double theta1_dot = t1 * vx + t2 * vy;
        double theta2_dot = t3 * vx + t4 * vy;
        double theta3_dot = t5 * vx + t6 * vy;

        thetaDot1 = theta1_dot;
        thetaDot2 = theta2_dot;
        thetaDot3 = theta3_dot;

        theta1 += theta1_dot * dt;
        theta2 += theta2_dot * dt;
        theta3 += theta3_dot * dt;

        enforceLimits();
    }

    public double[] endEffectorPosition() {
        double[] pos = computeAllPositions();
        return new double[]{pos[4], pos[5]};
    }
}