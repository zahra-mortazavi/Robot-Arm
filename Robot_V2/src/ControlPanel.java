import javax.swing.*;
import javax.swing.border.TitledBorder;
import javax.swing.table.AbstractTableModel;
import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

public class ControlPanel extends JPanel {

    private RobotArm robot;

    JointAngleTableModel angleModel;
    JointVelocityTableModel velModel;
    JointLinearVelocityTableModel linVelModel;
    EndEffectorTableModel eeModel;

    private double lastTheta1 = 0.0;
    private double lastTheta2 = 0.0;
    private double lastTheta3 = 0.0;
    private long lastTime;

    private double prevCsvTheta1 = 0.0;
    private double prevCsvTheta2 = 0.0;
    private double prevCsvTheta3 = 0.0;
    private double prevEeX = 0.0;
    private double prevEeY = 0.0;

    private PrintWriter csvWriter;
    private long startTime;

    public ControlPanel(boolean isManual) {
        setLayout(new BorderLayout());
    }

    public void attachRobot(RobotArm robot) {
        this.robot = robot;

        this.lastTheta1 = robot.theta1;
        this.lastTheta2 = robot.theta2;
        this.lastTheta3 = robot.theta3;
        this.lastTime = System.currentTimeMillis();

        this.prevCsvTheta1 = robot.theta1;
        this.prevCsvTheta2 = robot.theta2;
        this.prevCsvTheta3 = robot.theta3;
        double[] initialEE = robot.endEffectorPosition();
        this.prevEeX = initialEE[0];
        this.prevEeY = initialEE[1];

        this.startTime = System.currentTimeMillis();

        Box tablesBox = Box.createVerticalBox();

        angleModel = new JointAngleTableModel(robot);
        velModel = new JointVelocityTableModel(robot);
        linVelModel = new JointLinearVelocityTableModel(robot);
        eeModel = new EndEffectorTableModel(robot);

        tablesBox.add(createTable("Joint Angles", angleModel));
        tablesBox.add(createTable("Joint Angular Velocity", velModel));
        tablesBox.add(createTable("Joint Linear Velocity", linVelModel));
        tablesBox.add(createTable("End Effector", eeModel));

        tablesBox.add(Box.createVerticalGlue());

        JScrollPane scrollPane = new JScrollPane(tablesBox);
        scrollPane.setBorder(null);
        add(scrollPane, BorderLayout.CENTER);

        try {
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
            csvWriter = new PrintWriter(new FileWriter("robot_data_" + timestamp + ".csv", true));
            csvWriter.println("Time(s),Theta1,Theta2,Theta3,ThetaDot1,ThetaDot2,ThetaDot3,J1_Vx,J1_Vy,J2_Vx,J2_Vy,J3_Vx,J3_Vy,EE_X,EE_Y,EE_Vx,EE_Vy,EE_V_Mag");

            csvWriter.flush();
        } catch (IOException ex) {
            ex.printStackTrace();
        }

        Timer uiTimer = new Timer(100, e -> {
            if (this.robot != null) {
                long currentTime = System.currentTimeMillis();
                double dt = (currentTime - lastTime) / 1000.0;

                if (dt > 0) {
                    robot.thetaDot1 = getShortestAngularDistance(lastTheta1, robot.theta1) / dt;
                    robot.thetaDot2 = getShortestAngularDistance(lastTheta2, robot.theta2) / dt;
                    robot.thetaDot3 = getShortestAngularDistance(lastTheta3, robot.theta3) / dt;
                }

                lastTheta1 = robot.theta1;
                lastTheta2 = robot.theta2;
                lastTheta3 = robot.theta3;
                lastTime = currentTime;
            }

            angleModel.refresh();
            velModel.refresh();
            linVelModel.refresh();
            eeModel.refresh();
        });
        uiTimer.start();

        Timer csvTimer = new Timer(3000, e -> {
            if (this.robot != null && csvWriter != null) {
                double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

                double realThetaDot1 = getShortestAngularDistance(prevCsvTheta1, robot.theta1) / 3.0;
                double realThetaDot2 = getShortestAngularDistance(prevCsvTheta2, robot.theta2) / 3.0;
                double realThetaDot3 = getShortestAngularDistance(prevCsvTheta3, robot.theta3) / 3.0;

                double[] currentEE = robot.endEffectorPosition();
                double eeVx = (currentEE[0] - prevEeX) / 3.0;
                double eeVy = (currentEE[1] - prevEeY) / 3.0;
                double eeVMag = Math.sqrt(eeVx * eeVx + eeVy * eeVy);

                double tempDot1 = robot.thetaDot1;
                double tempDot2 = robot.thetaDot2;

                robot.thetaDot1 = realThetaDot1;
                robot.thetaDot2 = realThetaDot2;

                double[] v1 = robot.joint1Velocity();
                double[] v2 = robot.joint2Velocity();

                robot.thetaDot1 = tempDot1;
                robot.thetaDot2 = tempDot2;

                String row = String.format(
                        "%.3f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                        elapsedTime,
                        robot.theta1, robot.theta2, robot.theta3,
                        realThetaDot1, realThetaDot2, realThetaDot3,
                        v1[0], v1[1],
                        v2[0], v2[1],
                        eeVx, eeVy,
                        currentEE[0], currentEE[1],
                        eeVx, eeVy,
                        eeVMag
                );

                csvWriter.println(row);
                csvWriter.flush();

                prevCsvTheta1 = robot.theta1;
                prevCsvTheta2 = robot.theta2;
                prevCsvTheta3 = robot.theta3;
                prevEeX = currentEE[0];
                prevEeY = currentEE[1];
            }
        });
        csvTimer.start();
    }

    private JPanel createTable(String title, AbstractTableModel model) {
        JTable table = new JTable(model);
        table.setRowHeight(20);
        JPanel panel = new JPanel(new BorderLayout());
        panel.setBorder(new TitledBorder(title));
        JScrollPane scroll = new JScrollPane(table);
        scroll.setPreferredSize(new Dimension(250, 100));
        panel.add(scroll, BorderLayout.CENTER);
        panel.setMaximumSize(new Dimension(Integer.MAX_VALUE, 120));
        return panel;
    }

    private double getShortestAngularDistance(double angle1, double angle2) {
        double diff = (angle2 - angle1) % (2 * Math.PI);
        if (diff > Math.PI) {
            diff -= 2 * Math.PI;
        } else if (diff < -Math.PI) {
            diff += 2 * Math.PI;
        }
        return diff;
    }
}
