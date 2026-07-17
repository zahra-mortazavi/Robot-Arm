import java.util.HashMap;
import java.util.Map;
import java.util.Random;

public class RLController {
    private final Map<String, double[]> qTable = new HashMap<>();
    private final double[][] actions = new double[27][3];

    private final double actionStep = 0.01;

    public RLController() {
        int idx = 0;
        double[] choices = {-actionStep, 0, actionStep};
        for (double a1 : choices) {
            for (double a2 : choices) {
                for (double a3 : choices) {
                    actions[idx++] = new double[]{a1, a2, a3};
                }
            }
        }
    }

    private String getStateKey(RobotArm arm, double targetX, double targetY) {
        double[] p = arm.forwardKinematics();
        double currentX = p[6];
        double currentY = p[7];

        double dx = targetX - currentX;
        double dy = targetY - currentY;


        int dxBin = (int) (dx / 0.02);
        int dyBin = (int) (dy / 0.02);

        int q1Bin = (int) (arm.theta1 / 0.05);
        int q2Bin = (int) (arm.theta2 / 0.05);
        int q3Bin = (int) (arm.theta3 / 0.05);

        return dxBin + "_" + dyBin + "_" + q1Bin + "_" + q2Bin + "_" + q3Bin;
    }

    private double[] getQValues(String stateKey) {
        return qTable.computeIfAbsent(stateKey, k -> new double[27]);
    }

    public void train(double A1, double A2, double A3,
                      double j1Min, double j1Max, double j2Min, double j2Max, double j3Min, double j3Max,
                      Path path) {

        Random random = new Random(42);
        double alpha = 0.5;
        double gamma = 0.99;
        double epsilon = 0.7;

        int episodes = 20000;
        int stepsPerEpisode = 100;

        System.out.println("Training RL Agent directly on Path Coordinates...");

        for (int ep = 0; ep < episodes; ep++) {
            RobotArm mockArm = new RobotArm(A1, A2, A3, j1Min, j1Max, j2Min, j2Max, j3Min, j3Max);

            double s = random.nextDouble();
            double[] target = path.getPoint(s);

            try {
                double[] inverseAngles = mockArm.computeIKRaw(target[0], target[1], 0);
                mockArm.theta1 = inverseAngles[0] + (random.nextDouble() * 0.1 - 0.05);
                mockArm.theta2 = inverseAngles[1] + (random.nextDouble() * 0.1 - 0.05);
                mockArm.theta3 = inverseAngles[2] + (random.nextDouble() * 0.1 - 0.05);
            } catch (Exception e) {
                mockArm.theta1 = j1Min + random.nextDouble() * (j1Max - j1Min);
                mockArm.theta2 = j2Min + random.nextDouble() * (j2Max - j2Min);
                mockArm.theta3 = j3Min + random.nextDouble() * (j3Max - j3Min);
            }
            mockArm.enforceLimits();

            double[] initialP = mockArm.forwardKinematics();
            double prevDistance = Math.hypot(target[0] - initialP[6], target[1] - initialP[7]);

            for (int step = 0; step < stepsPerEpisode; step++) {
                String stateKey = getStateKey(mockArm, target[0], target[1]);
                double[] qValues = getQValues(stateKey);

                int actionIdx;
                if (random.nextDouble() < epsilon) {
                    actionIdx = random.nextInt(27);
                } else {
                    actionIdx = getBestActionIndex(qValues);
                }

                double[] action = actions[actionIdx];
                mockArm.theta1 += action[0];
                mockArm.theta2 += action[1];
                mockArm.theta3 += action[2];

                mockArm.enforceLimits();

                double[] p = mockArm.forwardKinematics();
                double distance = Math.hypot(target[0] - p[6], target[1] - p[7]);

                double reward = (prevDistance - distance) * 100.0;
                reward -= 0.1;

                boolean done = false;
                if (distance < 0.05) {
                    reward += 100.0;
                    done = true;
                }

                String nextStateKey = getStateKey(mockArm, target[0], target[1]);
                double[] nextQValues = getQValues(nextStateKey);
                int bestNextAction = getBestActionIndex(nextQValues);

                qValues[actionIdx] += alpha * (reward + gamma * nextQValues[bestNextAction] - qValues[actionIdx]);

                prevDistance = distance;

                if (done) {
                    break;
                }
            }

            if (ep % 500 == 0 && epsilon > 0.01) {
                epsilon *= 0.85;
            }
        }
        System.out.println("RL Training finished. Q-Table Size: " + qTable.size());
    }

    public void executePolicyStep(RobotArm arm, double[] target) {
        String stateKey = getStateKey(arm, target[0], target[1]);

        if (!qTable.containsKey(stateKey)) {
            moveStepGreedy(arm, target);
            return;
        }

        double[] qValues = qTable.get(stateKey);
        int bestActionIdx = getBestActionIndex(qValues);

        double[] action = actions[bestActionIdx];
        arm.theta1 += action[0];
        arm.theta2 += action[1];
        arm.theta3 += action[2];


        arm.enforceLimits();

        arm.thetaDot1 = action[0] / 0.04;
        arm.thetaDot2 = action[1] / 0.04;
        arm.thetaDot3 = action[2] / 0.04;
    }

    private void moveStepGreedy(RobotArm arm, double[] target) {
        double[] currentP = arm.forwardKinematics();
        double currentDist = Math.hypot(target[0] - currentP[6], target[1] - currentP[7]);

        int bestIdx = 13;
        double minDist = currentDist;

        double initT1 = arm.theta1;
        double initT2 = arm.theta2;
        double initT3 = arm.theta3;

        for (int i = 0; i < actions.length; i++) {
            double[] a = actions[i];
            arm.theta1 = initT1 + a[0];
            arm.theta2 = initT2 + a[1];
            arm.theta3 = initT3 + a[2];

            arm.enforceLimits();

            double[] p = arm.forwardKinematics();
            double d = Math.hypot(target[0] - p[6], target[1] - p[7]);

            if (d < minDist) {
                minDist = d;
                bestIdx = i;
            }
        }


        double[] bestAction = actions[bestIdx];
        arm.theta1 = initT1 + bestAction[0];
        arm.theta2 = initT2 + bestAction[1];
        arm.theta3 = initT3 + bestAction[2];
        arm.enforceLimits();
    }

    private int getBestActionIndex(double[] qValues) {
        int bestIdx = 0;
        double maxVal = qValues[0];
        for (int i = 1; i < qValues.length; i++) {
            if (qValues[i] > maxVal) {
                maxVal = qValues[i];
                bestIdx = i;
            }
        }
        return bestIdx;
    }
}
