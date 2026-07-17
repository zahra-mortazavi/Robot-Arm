import javax.swing.*;
import java.awt.*;

public class RobotSimulatorFrame extends JFrame {

    private static final double AUTOMATIC_GAIN = 0.185;
    private static final double AUTOMATIC_THRESHOLD = 0.02;
    private static final double AUTOMATIC_STEP = 0.003;
    private final double DYNAMIC_KP = 50.0;
    private final double DYNAMIC_KI = 0.5;
    private final double DYNAMIC_KD = 10.0;


    private static final double DYNAMIC_THRESHOLD = 0.06;
    private static final double DYNAMIC_STEP = 0.003;

    private static final double RL_THRESHOLD = 0.05;
    private static final double RL_STEP = 0.005;

    private final RobotArm robot;
    private final RobotPanel robotPanel;
    private final ControlPanel controlPanel;
    private MiniRobotPanel miniPanel;

    private Path path;
    private double s = 0.0;
    private boolean reachedPathStart = false;

    public enum Mode { AUTOMATIC, MANUAL, RL_AUTOMATIC, DYNAMIC_AUTOMATIC }
    private final Mode mode;

    private RLController rlController;
    private DynamicController dynamicController;

    public RobotSimulatorFrame(
            double A1, double A2, double A3,
            String pathType,
            double X1, double Y1,
            double X2, double Y2,
            double CX, double CY, double R,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max,
            String pressure,
            boolean manualModeEnabled,
            boolean rlModeEnabled,
            boolean dynamicModeEnabled,
            RLController trainedController
    ) {
        setTitle("Robot Simulator");

        if (manualModeEnabled) {
            this.mode = Mode.MANUAL;
        } else if (rlModeEnabled) {
            this.mode = Mode.RL_AUTOMATIC;
            this.rlController = trainedController;
        } else if (dynamicModeEnabled) {
            this.mode = Mode.DYNAMIC_AUTOMATIC;
            dynamicController = new DynamicController(DYNAMIC_KP, DYNAMIC_KI, DYNAMIC_KD, 0.05);
        } else {
            this.mode = Mode.AUTOMATIC;
        }

        robot = new RobotArm(A1, A2, A3, j1Min, j1Max, j2Min, j2Max, j3Min, j3Max);

        if (this.mode != Mode.MANUAL) {
            if ("Line".equals(pathType)) {
                path = new LinePath(X1, Y1, X2, Y2);
            } else {
                path = new CirclePath(CX, CY, R);
            }
        }

        controlPanel = new ControlPanel(this.mode == Mode.MANUAL);

        robotPanel = new RobotPanel(robot);
        robotPanel.setPressure(pressure);

        if (this.mode != Mode.MANUAL) {
            robotPanel.setTargetPath(path);
        }

        setLayout(new BorderLayout());
        add(robotPanel, BorderLayout.CENTER);

        controlPanel.attachRobot(robot);
        add(controlPanel, BorderLayout.EAST);

        if (this.mode == Mode.MANUAL) {
            miniPanel = new MiniRobotPanel(robot, () -> {
                robotPanel.updateTrail();
                robotPanel.repaint();
            });
            add(miniPanel, BorderLayout.SOUTH);
        }

        Timer timer = new Timer(40, e -> updateSimulation());
        timer.start();
        JButton endButton = new JButton("End / Back to Menu");
        endButton.addActionListener(e -> {
            if (timer.isRunning()) {
                timer.stop();
            }
            this.dispose();
            new PreStartFrame().setVisible(true);
        });

        JPanel topPanel = new JPanel();
        topPanel.add(endButton);
        add(topPanel, BorderLayout.WEST);
        setSize(950, 700);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setVisible(true);
    }

    void updateSimulation() {
        if (mode == Mode.MANUAL || path == null) {
            return;
        }

        if (mode == Mode.AUTOMATIC) {
            double[] target = getTargetPoint();

            for (int i = 0; i < 5; i++) {
                robot.jacobianControlStep(target[0], target[1], AUTOMATIC_GAIN);
            }

            double[] p = robot.forwardKinematics();
            updateProgress(target, p[6], p[7], AUTOMATIC_THRESHOLD, AUTOMATIC_STEP);

        } else if (mode == Mode.DYNAMIC_AUTOMATIC) {
            double[] target = getTargetPoint();

            try {

                double[] targetAngles = getSafeTargetAngles(robot, target[0], target[1]);

                if (targetAngles != null) {
                    double[] newAngles = dynamicController.updateAngles(
                            robot.theta1, robot.theta2, robot.theta3,
                            targetAngles[0], targetAngles[1], targetAngles[2]
                    );

                    robot.theta1 = newAngles[0];
                    robot.theta2 = newAngles[1];
                    robot.theta3 = newAngles[2];
                    robot.enforceLimits();
                }
            } catch (Exception e) {
                System.out.println("نقطه خارج از دسترس: " + e.getMessage());
            }

            double[] p = robot.forwardKinematics();
            updateProgress(target, p[6], p[7], DYNAMIC_THRESHOLD, DYNAMIC_STEP);

        } else if (mode == Mode.RL_AUTOMATIC) {
            double[] target = getTargetPoint();

            for (int i = 0; i < 5; i++) {
                rlController.executePolicyStep(robot, target);
            }

            double[] p = robot.forwardKinematics();
            updateProgress(target, p[6], p[7], RL_THRESHOLD, RL_STEP);
        }

        robotPanel.updateTrail();
        robotPanel.repaint();
    }


    private double[] getSafeTargetAngles(RobotArm robot, double targetX, double targetY) {
        double[] bestAngles = null;
        double minDiff = Double.MAX_VALUE;


        for (double phi = -Math.PI; phi <= Math.PI; phi += 0.1) {
            try {
                double[] angles = robot.computeIKRaw(targetX, targetY, phi);
                if (angles != null) {

                    double diff = Math.pow(angles[0] - robot.theta1, 2) +
                            Math.pow(angles[1] - robot.theta2, 2) +
                            Math.pow(angles[2] - robot.theta3, 2);

                    if (diff < minDiff) {
                        minDiff = diff;
                        bestAngles = angles;
                    }
                }
            } catch (Exception ignored) {

            }
        }
        return bestAngles;
    }

    private double[] getTargetPoint() {
        return reachedPathStart ? path.getPoint(s) : path.getPoint(0);
    }

    private void updateProgress(double[] target, double xe, double ye, double threshold, double increment) {
        double err = Math.hypot(target[0] - xe, target[1] - ye);

        if (!reachedPathStart) {
            if (err < threshold) {
                reachedPathStart = true;
                s = 0.0;
            }
            return;
        }

        if (err < threshold) {
            s += increment;
            if (s > 1.0) {
                s = 0.0;
            }
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new PreStartFrame());
    }
}
