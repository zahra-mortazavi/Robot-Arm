import javax.swing.*;
import java.awt.*;

public class RobotSimulatorFrame extends JFrame {

    RobotArm robot;
    RobotPanel robotPanel;

    ControlPanel controlPanel;
    MiniRobotPanel miniPanel;

    Path path;
    double s = 0;

    public enum Mode { AUTOMATIC, MANUAL }
    private Mode mode;

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
            boolean manualModeEnabled
    )
    {
        setTitle("Robot Simulator");

        this.mode = manualModeEnabled ? Mode.MANUAL : Mode.AUTOMATIC;

        robot = new RobotArm(
                A1, A2, A3,
                j1Min, j1Max,
                j2Min, j2Max,
                j3Min, j3Max
        );

        if (!manualModeEnabled) {
            if (pathType.equals("Line"))
                path = new LinePath(X1, Y1, X2, Y2);
            else
                path = new CirclePath(CX, CY, R);
        }

        controlPanel = new ControlPanel(manualModeEnabled);
        robotPanel = new RobotPanel(robot);
        robotPanel.setPressure(pressure);


        if (!manualModeEnabled)
            robotPanel.setTargetPath(path);




        setLayout(new BorderLayout());

        add(robotPanel, BorderLayout.CENTER);
        controlPanel.attachRobot(robot);
        add(controlPanel, BorderLayout.EAST);



        if (mode == Mode.MANUAL) {

            miniPanel = new MiniRobotPanel(robot, () -> {
                robotPanel.repaint();

            });

            JPanel southContainer = new JPanel(new FlowLayout(FlowLayout.CENTER));
            southContainer.add(miniPanel);

            add(southContainer, BorderLayout.SOUTH);
        }

        Timer timer = new Timer(40, e -> updateSimulation());
        timer.start();

        setSize(950, 700);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setVisible(true);
    }

    void updateSimulation() {

        if (mode == Mode.AUTOMATIC) {

            double[] target = path.getPoint(s);

            for (int i = 0; i < 5; i++) {
                robot.jacobianControlStep(target[0], target[1], 0.185);
            }

            double[] p = robot.forwardKinematics();
            double xe = p[6];
            double ye = p[7];

            double err = Math.hypot(target[0] - xe, target[1] - ye);

            if (err < 0.02) {
                s += 0.003;
                if (s > 1) s = 0;
            }
        }

        robotPanel.updateTrail();
        robotPanel.repaint();

    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new PreStartFrame());
    }
}
