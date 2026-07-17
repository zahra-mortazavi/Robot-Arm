public class DynamicController {

    private double dTheta1 = 0;
    private double dTheta2 = 0;
    private double dTheta3 = 0;


    private double iError1 = 0;
    private double iError2 = 0;
    private double iError3 = 0;

    private final double Kp;
    private final double Ki;
    private final double Kd;
    private final double dt;

    private final double MAX_I_ERROR = 5.0;

    public DynamicController(double Kp, double Ki, double Kd, double dt) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.dt = dt;
    }

    public double[] updateAngles(
            double currentT1, double currentT2, double currentT3,
            double targetT1, double targetT2, double targetT3) {

        double e1 = targetT1 - currentT1;
        double e2 = targetT2 - currentT2;
        double e3 = targetT3 - currentT3;

        iError1 += e1 * dt;
        iError2 += e2 * dt;
        iError3 += e3 * dt;

        iError1 = Math.max(-MAX_I_ERROR, Math.min(MAX_I_ERROR, iError1));
        iError2 = Math.max(-MAX_I_ERROR, Math.min(MAX_I_ERROR, iError2));
        iError3 = Math.max(-MAX_I_ERROR, Math.min(MAX_I_ERROR, iError3));

        double ddTheta1 = Kp * e1 + Ki * iError1 - Kd * dTheta1;
        double ddTheta2 = Kp * e2 + Ki * iError2 - Kd * dTheta2;
        double ddTheta3 = Kp * e3 + Ki * iError3 - Kd * dTheta3;

        dTheta1 += ddTheta1 * dt;
        dTheta2 += ddTheta2 * dt;
        dTheta3 += ddTheta3 * dt;

        double newT1 = currentT1 + dTheta1 * dt;
        double newT2 = currentT2 + dTheta2 * dt;
        double newT3 = currentT3 + dTheta3 * dt;

        return new double[]{newT1, newT2, newT3};
    }

    public void reset() {
        dTheta1 = 0; dTheta2 = 0; dTheta3 = 0;
        iError1 = 0; iError2 = 0; iError3 = 0;
    }
}
