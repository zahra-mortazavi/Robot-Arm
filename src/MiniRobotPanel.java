import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Point2D;

public class MiniRobotPanel extends JPanel {

    private RobotArm robot;
    private Runnable onMove;

    private double scale = 33;
    private int dragRadius = 14;

    public MiniRobotPanel(RobotArm robot, Runnable onMove) {
        this.robot = robot;
        this.onMove = onMove;

        setPreferredSize(new Dimension(220, 220));
        setBackground(Color.WHITE);

        MouseAdapter ma = new MouseAdapter() {

            @Override
            public void mousePressed(MouseEvent e) {
                handleDrag(e);
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                handleDrag(e);
            }

            private void handleDrag(MouseEvent e) {
                int cx = getWidth() / 2;
                int cy = getHeight() / 2;

                double wx = (e.getX() - cx) / scale;
                double wy = (cy - e.getY()) / scale;

                robot.manualTarget(wx, wy, robot.theta1 + robot.theta2 + robot.theta3);

                if (onMove != null) {
                    onMove.run();
                }

                repaint();
            }
        };

        addMouseListener(ma);
        addMouseMotionListener(ma);
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int cx = getWidth() / 2;
        int cy = getHeight() / 2;


        g2.setColor(new Color(230, 230, 230));
        g2.drawLine(0, cy, getWidth(), cy);
        g2.drawLine(cx, 0, cx, getHeight());


        double[] p = robot.forwardKinematics();

        int x0 = cx;
        int y0 = cy;

        int x1 = (int) (cx + p[2] * scale);
        int y1 = (int) (cy - p[3] * scale);

        int x2 = (int) (cx + p[4] * scale);
        int y2 = (int) (cy - p[5] * scale);

        int x3 = (int) (cx + p[6] * scale);
        int y3 = (int) (cy - p[7] * scale);

        g2.setStroke(new BasicStroke(4));
        g2.setColor(Color.BLUE);
        g2.drawLine(x0, y0, x1, y1);
        g2.drawLine(x1, y1, x2, y2);
        g2.drawLine(x2, y2, x3, y3);

        g2.setColor(Color.BLACK);
        g2.fillOval(x0 - 4, y0 - 4, 8, 8);
        g2.fillOval(x1 - 4, y1 - 4, 8, 8);
        g2.fillOval(x2 - 4, y2 - 4, 8, 8);

        g2.setColor(Color.RED);
        g2.fillOval(x3 - 6, y3 - 6, 12, 12);

        g2.setColor(Color.DARK_GRAY);
        g2.drawString("Drag end-effector", 10, 20);
    }
}
