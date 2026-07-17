public class ReachabilityChecker {

    private static final double MAX_JOINT_STEP = Math.toRadians(45.0);

    public static class ReachCheckResult {
        public boolean reachable;
        public String message;
        public double failX;
        public double failY;
        public double[] joints;

        public ReachCheckResult(boolean reachable, String message, double failX, double failY, double[] joints) {
            this.reachable = reachable;
            this.message = message;
            this.failX = failX;
            this.failY = failY;
            this.joints = joints;
        }
    }

    public static ReachCheckResult checkLineReachDetailed(
            double A1, double A2, double A3,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max,
            double X1, double Y1,
            double X2, double Y2)
    {
        int samples = 200;

        RobotArm arm = new RobotArm(
                A1, A2, A3,
                j1Min, j1Max,
                j2Min, j2Max,
                j3Min, j3Max
        );

        double[] prevJoints = null;

        for (int i = 0; i <= samples; i++) {
            double t = (double) i / samples;

            double px = X1 + t * (X2 - X1);
            double py = Y1 + t * (Y2 - Y1);

            ReachCheckResult result = checkSinglePointReach(
                    arm,
                    px, py,
                    A1, A2, A3,
                    j1Min, j1Max,
                    j2Min, j2Max,
                    j3Min, j3Max,
                    prevJoints
            );

            if (!result.reachable) {
                return result;
            }
            prevJoints = result.joints;
        }

        return new ReachCheckResult(
                true,
                "Line is reachable.",
                Double.NaN,
                Double.NaN,
                null
        );
    }

    public static ReachCheckResult checkCircleReachDetailed(
            double A1, double A2, double A3,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max,
            double CX, double CY, double R)
    {
        int samples = 200;

        RobotArm arm = new RobotArm(
                A1, A2, A3,
                j1Min, j1Max,
                j2Min, j2Max,
                j3Min, j3Max
        );

        double[] prevJoints = null;

        for (int i = 0; i < samples; i++) {
            double angle = (2 * Math.PI * i) / samples;

            double px = CX + R * Math.cos(angle);
            double py = CY + R * Math.sin(angle);

            ReachCheckResult result = checkSinglePointReach(
                    arm,
                    px, py,
                    A1, A2, A3,
                    j1Min, j1Max,
                    j2Min, j2Max,
                    j3Min, j3Max,
                    prevJoints
            );

            if (!result.reachable) {
                return result;
            }
            prevJoints = result.joints;
        }

        return new ReachCheckResult(
                true,
                "Circle is reachable.",
                Double.NaN,
                Double.NaN,
                null
        );
    }

    private static ReachCheckResult checkSinglePointReach(
            RobotArm arm,
            double x, double y,
            double A1, double A2, double A3,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max,
            double[] prevJoints)
    {
        double d = Math.hypot(x, y);

        double maxReach = A1 + A2 + A3;

        double longest = Math.max(A1, Math.max(A2, A3));
        double sumOthers = A1 + A2 + A3 - longest;
        double minReach = Math.max(0.0, longest - sumOthers);

        if (d > maxReach) {
            return new ReachCheckResult(
                    false,
                    "Point is outside maximum reach. Distance = " +
                            String.format("%.3f", d) +
                            ", Max reach = " +
                            String.format("%.3f", maxReach),
                    x,
                    y,
                    null
            );
        }

        if (d < minReach) {
            return new ReachCheckResult(
                    false,
                    "Point is inside unreachable inner workspace. Distance = " +
                            String.format("%.3f", d) +
                            ", Min reach = " +
                            String.format("%.3f", minReach),
                    x,
                    y,
                    null
            );
        }

        double[] solution = findValidIKSolution(
                arm,
                x, y,
                j1Min, j1Max,
                j2Min, j2Max,
                j3Min, j3Max,
                A1, A2, A3,
                prevJoints
        );

        if (solution == null) {
            String msg = (prevJoints == null)
                    ? "No valid IK solution inside joint limits."
                    : "No continuous valid IK solution found (Joint jump detected).";
            return new ReachCheckResult(
                    false,
                    msg,
                    x,
                    y,
                    null
            );
        }

        return new ReachCheckResult(
                true,
                "OK",
                x,
                y,
                solution
        );
    }

    private static double[] findValidIKSolution(
            RobotArm arm,
            double x, double y,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max,
            double A1, double A2, double A3,
            double[] prevJoints)
    {

        int phiSteps = 72;
        double[] phiSamples = new double[phiSteps];
        for (int i = 0; i < phiSteps; i++) {
            phiSamples[i] = -Math.PI + i * (2 * Math.PI / phiSteps);
        }


        double[] bestSolution = null;
        double minDiff = Double.MAX_VALUE;

        for (double phi : phiSamples) {
            try {
                double[] th = arm.computeIKRaw(x, y, phi);

                if (isValidConfiguration(th, A1, A2, A3, j1Min, j1Max, j2Min, j2Max, j3Min, j3Max)) {
                    if (prevJoints == null) return th;

                    double diff = calculateJointDifference(th, prevJoints);
                    if (diff <= MAX_JOINT_STEP && diff < minDiff) {
                        minDiff = diff;
                        bestSolution = th;
                    }
                }

                double[] thAlt = new double[]{
                        th[0],
                        -th[1],
                        phi - th[0] + th[1]
                };

                if (isValidConfiguration(thAlt, A1, A2, A3, j1Min, j1Max, j2Min, j2Max, j3Min, j3Max)) {
                    if (prevJoints == null) return thAlt;

                    double diff = calculateJointDifference(thAlt, prevJoints);
                    if (diff <= MAX_JOINT_STEP && diff < minDiff) {
                        minDiff = diff;
                        bestSolution = thAlt;
                    }
                }

            } catch (Exception ignored) {

            }
        }
        return bestSolution;
    }

    private static double calculateJointDifference(double[] current, double[] prev) {
        if (prev == null) return 0.0;

        double maxDiff = 0.0;
        for (int i = 0; i < 3; i++) {
            double diff = Math.abs(normalizeAngle(current[i] - prev[i]));
            if (diff > maxDiff) {
                maxDiff = diff;
            }
        }
        return maxDiff;
    }

    private static boolean isValidConfiguration(
            double[] th,
            double A1, double A2, double A3,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max)
    {
        if (th == null || th.length < 3) {
            return false;
        }

        if (!inLimits(
                th,
                j1Min, j1Max,
                j2Min, j2Max,
                j3Min, j3Max
        )) {
            return false;
        }

        if (hasSimpleSelfCollision(
                A1, A2, A3,
                th[0], th[1], th[2]
        )) {
            return false;
        }

        return true;
    }

    private static boolean inLimits(
            double[] th,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max)
    {
        if (th == null || th.length < 3) {
            return false;
        }

        double t1 = normalizeAngle(th[0]);
        double t2 = normalizeAngle(th[1]);
        double t3 = normalizeAngle(th[2]);

        return isAngleInRange(t1, j1Min, j1Max)
                && isAngleInRange(t2, j2Min, j2Max)
                && isAngleInRange(t3, j3Min, j3Max);
    }

    private static boolean isAngleInRange(double angle, double min, double max) {
        double eps = 1e-9;

        angle = normalizeAngle(angle);
        min = normalizeAngle(min);
        max = normalizeAngle(max);

        if (min <= max) {
            return angle >= min - eps && angle <= max + eps;
        } else {
            return angle >= min - eps || angle <= max + eps;
        }
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    private static boolean hasSimpleSelfCollision(
            double A1, double A2, double A3,
            double t1, double t2, double t3)
    {
        double x1 = A1 * Math.cos(t1);
        double y1 = A1 * Math.sin(t1);

        double x2 = x1 + A2 * Math.cos(t1 + t2);
        double y2 = y1 + A2 * Math.sin(t1 + t2);

        double x3 = x2 + A3 * Math.cos(t1 + t2 + t3);
        double y3 = y2 + A3 * Math.sin(t1 + t2 + t3);

        double endToBase = Math.hypot(x3, y3);

        double smallDistanceThreshold = 0.15;

        if (endToBase < smallDistanceThreshold) {
            return true;
        }

        if (Math.abs(t2) > Math.toRadians(170)) {
            return true;
        }

        if (Math.abs(t3) > Math.toRadians(170)) {
            return true;
        }

        return false;
    }
}
