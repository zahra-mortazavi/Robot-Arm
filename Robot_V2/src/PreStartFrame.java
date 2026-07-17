import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

public class PreStartFrame extends JFrame {

    private static class CollapsiblePanel extends JPanel {

        private boolean expanded = true;
        private final JPanel contentPanel;
        private final JLabel headerLabel;

        public CollapsiblePanel(String title, JPanel content) {

            setLayout(new BorderLayout());
            setBorder(BorderFactory.createEmptyBorder(5,5,5,5));

            headerLabel = new JLabel("▼ " + title);

            headerLabel.setOpaque(true);
            headerLabel.setBackground(new Color(66,133,244));
            headerLabel.setForeground(Color.WHITE);
            headerLabel.setBorder(
                    BorderFactory.createEmptyBorder(8,12,8,12)
            );

            contentPanel = content;
            contentPanel.setBackground(new Color(250,250,250));

            headerLabel.addMouseListener(new MouseAdapter() {
                @Override
                public void mouseClicked(MouseEvent e) {
                    toggle();
                }
            });

            add(headerLabel, BorderLayout.NORTH);
            add(contentPanel, BorderLayout.CENTER);
        }

        private void toggle() {

            expanded = !expanded;

            contentPanel.setVisible(expanded);

            headerLabel.setText(
                    (expanded ? "▼ " : "► ")
                            + headerLabel.getText().substring(2)
            );

            revalidate();
            repaint();
        }
    }
    private CardLayout cardLayout;
    private JPanel mainPanel;

    private JTextField A1Field = new JTextField("1.5");
    private JTextField A2Field = new JTextField("1.2");
    private JTextField A3Field = new JTextField("0.8");

    private JTextField j1MinField = new JTextField("-180");
    private JTextField j1MaxField = new JTextField("180");

    private JTextField j2MinField = new JTextField("-180");
    private JTextField j2MaxField = new JTextField("180");

    private JTextField j3MinField = new JTextField("-180");
    private JTextField j3MaxField = new JTextField("180");

    private JComboBox<String> modeCombo =
            new JComboBox<>(new String[]{
                    "Automatic",
                    "Manual",
                    "RL_Automatic" ,
                    "Dynamic_Automatic"
            });

    private JComboBox<String> pressureCombo =
            new JComboBox<>(new String[]{
                    "Low",
                    "Medium",
                    "High"
            });


    private JComboBox<String> pathCombo =
            new JComboBox<>(new String[]{
                    "Line",
                    "Circle"
            });

    private JTextField x1Field = new JTextField("1.0");
    private JTextField y1Field = new JTextField("1.0");

    private JTextField x2Field = new JTextField("2.0");
    private JTextField y2Field = new JTextField("1.5");

    private JTextField cxField = new JTextField("1.5");
    private JTextField cyField = new JTextField("1.5");
    private JTextField rField = new JTextField("0.7");

    private JPanel linePanel;
    private JPanel circlePanel;

    private JPanel automaticPanel;
    private JPanel manualPanel;

    public PreStartFrame() {

        super("Robot Configuration");

        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        setSize(520,700);

        setLocationRelativeTo(null);

        cardLayout = new CardLayout();

        mainPanel = new JPanel(cardLayout);

        mainPanel.add(buildStep1Panel(), "STEP1");
        mainPanel.add(buildStep2Panel(), "STEP2");

        add(mainPanel);

        setVisible(true);
    }

    private JPanel buildStep1Panel() {

        JPanel container = new JPanel();

        container.setLayout(
                new BoxLayout(container, BoxLayout.Y_AXIS)
        );

        container.setBackground(new Color(245,245,245));

        container.add(
                buildCollapsible(
                        "Robot Link Lengths",
                        buildLinkPanel()
                )
        );

        container.add(
                buildCollapsible(
                        "Joint Angle Limits",
                        buildJointLimitPanel()
                )
        );

        JPanel operationPanel = createForm();

        addRow(operationPanel, "Mode:", modeCombo);
        addRow(operationPanel, "Pressure:", pressureCombo);

        container.add(
                buildCollapsible(
                        "Operation Settings",
                        operationPanel
                )
        );

        JButton nextBtn = new JButton("Next");

        nextBtn.setBackground(new Color(52,168,83));
        nextBtn.setForeground(Color.WHITE);

        nextBtn.setFont(
                nextBtn.getFont().deriveFont(Font.BOLD,16)
        );

        nextBtn.addActionListener(e -> {
            String mode = (String) modeCombo.getSelectedItem();

            boolean isAutomatic =
                    mode.equals("Automatic")
                            || mode.equals("Dynamic_Automatic")
                            || mode.equals("RL_Automatic");

            automaticPanel.setVisible(isAutomatic);
            manualPanel.setVisible(!isAutomatic);

            mainPanel.revalidate();
            mainPanel.repaint();


            if (mode.equalsIgnoreCase("Manual")) {

                startSimulation();
            } else {

                cardLayout.show(mainPanel, "STEP2");
            }
        });


        JPanel btnPanel = new JPanel();
        btnPanel.add(nextBtn);

        container.add(btnPanel);

        JScrollPane scrollPane =
                new JScrollPane(container);

        scrollPane.setBorder(null);

        JPanel wrapper = new JPanel(new BorderLayout());
        wrapper.add(scrollPane, BorderLayout.CENTER);

        return wrapper;
    }

    private JPanel buildStep2Panel() {

        JPanel root = new JPanel(new BorderLayout());

        JPanel content = new JPanel();

        content.setLayout(
                new BoxLayout(content, BoxLayout.Y_AXIS)
        );

        automaticPanel = buildAutomaticPanel();
        manualPanel = buildManualPanel();

        automaticPanel.setVisible(true);
        manualPanel.setVisible(false);

        content.add(automaticPanel);
        content.add(manualPanel);

        JScrollPane scrollPane =
                new JScrollPane(content);

        scrollPane.setBorder(null);

        root.add(scrollPane, BorderLayout.CENTER);

        JPanel bottomPanel = new JPanel();

        JButton backBtn = new JButton("Back");
        backBtn.setBackground(new Color(52,168,83));

        JButton startBtn = new JButton("Start Simulation");
        startBtn.setBackground(new Color(52,168,83));

        backBtn.addActionListener(e ->
                cardLayout.show(mainPanel,"STEP1")
        );

        startBtn.addActionListener(e ->
                startSimulation()
        );

        bottomPanel.add(backBtn);
        bottomPanel.add(startBtn);

        root.add(bottomPanel, BorderLayout.SOUTH);

        return root;
    }

    private JPanel buildAutomaticPanel() {

        JPanel root = new JPanel();

        root.setLayout(
                new BoxLayout(root, BoxLayout.Y_AXIS)
        );

        JPanel top = createForm();

        addRow(top, "Path Type:", pathCombo);

        root.add(
                buildCollapsible(
                        "Automatic Settings",
                        top
                )
        );

        linePanel = createForm();

        addRow(linePanel, "X1:", x1Field);
        addRow(linePanel, "Y1:", y1Field);

        addRow(linePanel, "X2:", x2Field);
        addRow(linePanel, "Y2:", y2Field);

        circlePanel = createForm();

        addRow(circlePanel, "Center X:", cxField);
        addRow(circlePanel, "Center Y:", cyField);

        addRow(circlePanel, "Radius:", rField);

        root.add(
                buildCollapsible(
                        "Line Parameters",
                        linePanel
                )
        );

        root.add(
                buildCollapsible(
                        "Circle Parameters",
                        circlePanel
                )
        );

        updatePathPanels();

        pathCombo.addActionListener(e ->
                updatePathPanels()
        );

        return root;
    }

    private JPanel buildManualPanel() {

        JPanel p = new JPanel();

        p.setLayout(
                new BoxLayout(p, BoxLayout.Y_AXIS)
        );



        return p;
    }

    private void updatePathPanels() {

        String type =
                (String) pathCombo.getSelectedItem();

        boolean isLine = type.equals("Line");

        linePanel.setVisible(isLine);

        circlePanel.setVisible(!isLine);

        revalidate();
        repaint();
    }


    private JPanel buildLinkPanel() {

        JPanel p = createForm();

        addRow(p, "Link 1 Length:", A1Field);
        addRow(p, "Link 2 Length:", A2Field);
        addRow(p, "Link 3 Length:", A3Field);

        return p;
    }

    private JPanel buildJointLimitPanel() {

        JPanel p = createForm();

        addRow(p, "Joint 1 Min:", j1MinField);
        addRow(p, "Joint 1 Max:", j1MaxField);

        addRow(p, "Joint 2 Min:", j2MinField);
        addRow(p, "Joint 2 Max:", j2MaxField);

        addRow(p, "Joint 3 Min:", j3MinField);
        addRow(p, "Joint 3 Max:", j3MaxField);

        return p;
    }

    private JPanel createForm() {

        JPanel p = new JPanel(
                new GridLayout(0,2,10,10)
        );

        p.setBorder(
                BorderFactory.createEmptyBorder(
                        10,10,10,10
                )
        );

        p.setBackground(new Color(250,250,250));

        return p;
    }

    private void addRow(
            JPanel p,
            String label,
            JComponent field
    ) {

        JLabel l = new JLabel(label);

        l.setFont(
                l.getFont().deriveFont(Font.PLAIN,14)
        );

        p.add(l);

        p.add(field);
    }

    private CollapsiblePanel buildCollapsible(
            String title,
            JPanel content
    ) {

        return new CollapsiblePanel(title, content);
    }

    private void startSimulation() {

        try {

            double A1 = Double.parseDouble(A1Field.getText());
            double A2 = Double.parseDouble(A2Field.getText());
            double A3 = Double.parseDouble(A3Field.getText());

            double j1Min = Math.toRadians(Double.parseDouble(j1MinField.getText()));
            double j1Max = Math.toRadians(Double.parseDouble(j1MaxField.getText()));

            double j2Min = Math.toRadians(Double.parseDouble(j2MinField.getText()));
            double j2Max = Math.toRadians(Double.parseDouble(j2MaxField.getText()));

            double j3Min = Math.toRadians(Double.parseDouble(j3MinField.getText()));
            double j3Max = Math.toRadians(Double.parseDouble(j3MaxField.getText()));

            String mode = (String) modeCombo.getSelectedItem();
            String pressure = (String) pressureCombo.getSelectedItem();
            String pathType = (String) pathCombo.getSelectedItem();

            double X1 = Double.parseDouble(x1Field.getText());
            double Y1 = Double.parseDouble(y1Field.getText());
            double X2 = Double.parseDouble(x2Field.getText());
            double Y2 = Double.parseDouble(y2Field.getText());

            double CX = Double.parseDouble(cxField.getText());
            double CY = Double.parseDouble(cyField.getText());
            double R  = Double.parseDouble(rField.getText());

            if (mode.equals("Automatic") ||
                    mode.equals("RL_Automatic") ||
                    mode.equals("Dynamic_Automatic")) {

                double maxReach = A1 + A2 + A3;
                ReachabilityChecker.ReachCheckResult reachResult;

                if (pathType.equalsIgnoreCase("Line")) {
                    reachResult = ReachabilityChecker.checkLineReachDetailed(
                            A1, A2, A3, j1Min, j1Max, j2Min, j2Max, j3Min, j3Max, X1, Y1, X2, Y2
                    );

                    if (!reachResult.reachable) {
                        JOptionPane.showMessageDialog(this,
                                "Robot cannot follow this line.\n\nReason: " + reachResult.message + "\n\n" +
                                        String.format("Failed near point: X = %.3f , Y = %.3f", reachResult.failX, reachResult.failY),
                                "Reachability Error", JOptionPane.ERROR_MESSAGE);
                        return;
                    }
                } else if (pathType.equalsIgnoreCase("Circle")) {
                    reachResult = ReachabilityChecker.checkCircleReachDetailed(
                            A1, A2, A3, j1Min, j1Max, j2Min, j2Max, j3Min, j3Max, CX, CY, R
                    );

                    if (!reachResult.reachable) {
                        JOptionPane.showMessageDialog(this,
                                "Robot cannot draw this circle.\n\nReason: " + reachResult.message + "\n\n" +
                                        String.format("Failed near point: X = %.3f , Y = %.3f", reachResult.failX, reachResult.failY),
                                "Reachability Error", JOptionPane.ERROR_MESSAGE);
                        return;
                    }
                }
            }

            boolean manualModeEnabled = mode.equalsIgnoreCase("Manual");
            boolean rlModeEnabled = mode.equalsIgnoreCase("RL_Automatic");


            double bestStartAngle = 0.0;
            if (pathType.equalsIgnoreCase("Circle")) {
                RobotArm tempArm = new RobotArm(A1, A2, A3, j1Min, j1Max, j2Min, j2Max, j3Min, j3Max);
                double epsilon = 0.05;

                for (double angle = 0; angle < 360; angle += 1) {
                    double rad = Math.toRadians(angle);
                    double px = CX + R * Math.cos(rad);
                    double py = CY + R * Math.sin(rad);
                    try {
                        double[] ik = tempArm.computeIKRaw(px, py, 0.0);
                        double t1 = ik[0];

                        if (Math.abs(t1 - j1Min) < epsilon || Math.abs(t1 - j1Max) < epsilon) {
                            bestStartAngle = rad;
                            break;
                        }
                    } catch (Exception ignored) {}
                }
            }


            Path selectedPath = pathType.equals("Line") ? new LinePath(X1, Y1, X2, Y2) : new CirclePath(CX, CY, R, bestStartAngle);

            if (rlModeEnabled) {
                JDialog trainingDialog = new JDialog(this, "Training RL Agent", true);
                trainingDialog.setLayout(new FlowLayout(FlowLayout.CENTER, 10, 20));
                trainingDialog.add(new JLabel("Please wait, training the robot arm using RL..."));
                trainingDialog.setSize(350, 110);
                trainingDialog.setLocationRelativeTo(this);

                SwingWorker<RLController, Void> worker = new SwingWorker<>() {
                    @Override
                    protected RLController doInBackground() throws Exception {
                        RLController controller = new RLController();
                        controller.train(A1, A2, A3, j1Min, j1Max, j2Min, j2Max, j3Min, j3Max, selectedPath);
                        return controller;
                    }

                    @Override
                    protected void done() {
                        try {
                            RLController trainedController = get();
                            trainingDialog.dispose();

                            new RobotSimulatorFrame(
                                    A1, A2, A3, pathType, X1, Y1, X2, Y2, CX, CY, R,
                                    j1Min, j1Max, j2Min, j2Max, j3Min, j3Max,
                                    pressure, false, true, false, trainedController
                            );
                            dispose();

                        } catch (Exception ex) {
                            ex.printStackTrace();
                            JOptionPane.showMessageDialog(null, "Error during RL training: " + ex.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
                        }
                    }
                };

                worker.execute();
                trainingDialog.setVisible(true);

            } else {
                boolean dynamicModeEnabled = mode.equals("Dynamic_Automatic");

                new RobotSimulatorFrame(
                        A1, A2, A3, pathType, X1, Y1, X2, Y2, CX, CY, R,
                        j1Min, j1Max, j2Min, j2Max, j3Min, j3Max,
                        pressure, manualModeEnabled, false, dynamicModeEnabled, null
                );
                dispose();
            }

        } catch (Exception ex) {
            JOptionPane.showMessageDialog(this, "Invalid Input!", "Error", JOptionPane.ERROR_MESSAGE);
        }
    }

}