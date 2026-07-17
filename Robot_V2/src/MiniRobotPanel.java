import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

public class MiniRobotPanel extends JPanel {

    private RobotArm robot;
    private Runnable onMove;
    private DrawingArea drawingArea;

    public MiniRobotPanel(RobotArm robot, Runnable onMove) {
        this.robot = robot;
        this.onMove = onMove;


        setLayout(new FlowLayout(FlowLayout.CENTER, 15, 5));


        JPanel keysPanel = new JPanel(new GridLayout(3, 2, 5, 5));
        keysPanel.add(new JLabel("Q : J1+"));
        keysPanel.add(new JLabel("A : J1-"));
        keysPanel.add(new JLabel("W : J2+"));
        keysPanel.add(new JLabel("S : J2-"));
        keysPanel.add(new JLabel("E : J3+"));
        keysPanel.add(new JLabel("D : J3-"));
        add(keysPanel);


        drawingArea = new DrawingArea();
        add(drawingArea);


        JPanel buttonsPanel = new JPanel(new GridLayout(3, 2, 5, 5));
        double step = 0.05;

        JButton btnJ1Plus = new JButton("J1 +");
        JButton btnJ1Minus = new JButton("J1 -");
        JButton btnJ2Plus = new JButton("J2 +");
        JButton btnJ2Minus = new JButton("J2 -");
        JButton btnJ3Plus = new JButton("J3 +");
        JButton btnJ3Minus = new JButton("J3 -");

        btnJ1Plus.addActionListener(e -> moveJoint(1, step));
        btnJ1Minus.addActionListener(e -> moveJoint(1, -step));
        btnJ2Plus.addActionListener(e -> moveJoint(2, step));
        btnJ2Minus.addActionListener(e -> moveJoint(2, -step));
        btnJ3Plus.addActionListener(e -> moveJoint(3, step));
        btnJ3Minus.addActionListener(e -> moveJoint(3, -step));

        buttonsPanel.add(btnJ1Plus);
        buttonsPanel.add(btnJ1Minus);
        buttonsPanel.add(btnJ2Plus);
        buttonsPanel.add(btnJ2Minus);
        buttonsPanel.add(btnJ3Plus);
        buttonsPanel.add(btnJ3Minus);
        add(buttonsPanel);

        setupKeyboardControl();
    }

    private void moveJoint(int joint, double amount) {
        if (joint == 1) robot.theta1 += amount;
        else if (joint == 2) robot.theta2 += amount;
        else if (joint == 3) robot.theta3 += amount;

        robot.enforceLimits();
        if (onMove != null) {
            onMove.run();
        }
        drawingArea.repaint();
    }

    private void setupKeyboardControl() {
        final double step = 0.05;


        InputMap im = this.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
        ActionMap am = this.getActionMap();

        im.put(KeyStroke.getKeyStroke("pressed Q"), "incJoint1");
        im.put(KeyStroke.getKeyStroke("pressed A"), "decJoint1");
        im.put(KeyStroke.getKeyStroke("pressed W"), "incJoint2");
        im.put(KeyStroke.getKeyStroke("pressed S"), "decJoint2");
        im.put(KeyStroke.getKeyStroke("pressed E"), "incJoint3");
        im.put(KeyStroke.getKeyStroke("pressed D"), "decJoint3");

        am.put("incJoint1", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { moveJoint(1, step); }
        });
        am.put("decJoint1", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { moveJoint(1, -step); }
        });
        am.put("incJoint2", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { moveJoint(2, step); }
        });
        am.put("decJoint2", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { moveJoint(2, -step); }
        });
        am.put("incJoint3", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { moveJoint(3, step); }
        });
        am.put("decJoint3", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { moveJoint(3, -step); }
        });
    }


    private class DrawingArea extends JPanel {
        private double scale = 33;
        private int dragRadius = 14;

        public DrawingArea() {
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
}
