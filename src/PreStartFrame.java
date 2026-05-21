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
                    "Manual"
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
    private JTextField cyField = new JTextField("1.0");
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

            String mode =
                    (String) modeCombo.getSelectedItem();

            boolean isAutomatic =
                    mode.equals("Automatic");

            automaticPanel.setVisible(isAutomatic);
            manualPanel.setVisible(!isAutomatic);

            mainPanel.revalidate();
            mainPanel.repaint();

            cardLayout.show(mainPanel,"STEP2");
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

        JButton startBtn = new JButton("Start Simulation");

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

        JTextArea info = new JTextArea(
                "Manual Mode Enabled\n\n" +
                        "will add later:\n" +
                        "- Joint sliders\n" +
                        "- Keyboard control\n" +
                        "- Mouse control\n"
        );

        info.setEditable(false);

        info.setFont(new Font("Arial", Font.PLAIN,14));

        p.add(info);

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

    private boolean isReachableByJointLimitsAccurate(
            RobotArm arm,
            double x, double y,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max)
    {
        double[] phiSamples = {
                0,
                Math.PI/4,
                -Math.PI/4,
                Math.PI/2,
                -Math.PI/2,
                Math.PI,
                -Math.PI
        };

        for (double phi : phiSamples) {

            try {

                double[] th = arm.computeIKRaw(x, y, phi);

                if (inLimits(th,
                        j1Min, j1Max,
                        j2Min, j2Max,
                        j3Min, j3Max))
                    return true;

                double[] thAlt = new double[]{
                        th[0],
                        -th[1],
                        phi - th[0] - (-th[1])
                };

                if (inLimits(thAlt,
                        j1Min, j1Max,
                        j2Min, j2Max,
                        j3Min, j3Max))
                    return true;

            } catch (Exception ignored) {}
        }

        return false;
    }

    private boolean inLimits(
            double[] th,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max)
    {
        return (
                th[0] >= j1Min &&
                        th[0] <= j1Max &&

                        th[1] >= j2Min &&
                        th[1] <= j2Max &&

                        th[2] >= j3Min &&
                        th[2] <= j3Max
        );
    }

    private boolean checkLineAngularReach(
            double A1, double A2, double A3,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max,
            double X1, double Y1,
            double X2, double Y2)
    {
        int samples = 30;

        RobotArm arm = new RobotArm(
                A1, A2, A3,
                j1Min, j1Max,
                j2Min, j2Max,
                j3Min, j3Max
        );

        for (int i = 0; i <= samples; i++) {

            double t = (double) i / samples;

            double px = X1 + t * (X2 - X1);
            double py = Y1 + t * (Y2 - Y1);

            boolean ok =
                    isReachableByJointLimitsAccurate(
                            arm,
                            px,
                            py,
                            j1Min, j1Max,
                            j2Min, j2Max,
                            j3Min, j3Max
                    );

            if (!ok)
                return false;
        }

        return true;
    }


    private boolean checkCircleAngularReach(
            double A1, double A2, double A3,
            double j1Min, double j1Max,
            double j2Min, double j2Max,
            double j3Min, double j3Max,
            double CX, double CY, double R)
    {
        int samples = 60;

        RobotArm arm = new RobotArm(
                A1, A2, A3,
                j1Min, j1Max,
                j2Min, j2Max,
                j3Min, j3Max
        );

        for (int i = 0; i < samples; i++) {

            double angle =
                    (2 * Math.PI * i) / samples;

            double px =
                    CX + R * Math.cos(angle);

            double py =
                    CY + R * Math.sin(angle);

            boolean ok =
                    isReachableByJointLimitsAccurate(
                            arm,
                            px,
                            py,
                            j1Min, j1Max,
                            j2Min, j2Max,
                            j3Min, j3Max
                    );

            if (!ok)
                return false;
        }

        return true;
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


            if (mode.equals("Automatic")) {

                double maxReach = A1 + A2 + A3;




                if (pathType.equals("Line")) {

                    if (Math.hypot(X1, Y1) > maxReach ||
                            Math.hypot(X2, Y2) > maxReach)
                    {
                        JOptionPane.showMessageDialog(
                                this,
                                "Line path is out of robot reach.",
                                "Geometric Reach Error",
                                JOptionPane.ERROR_MESSAGE
                        );

                        return;
                    }

                } else {

                    if (Math.hypot(CX, CY) + R > maxReach) {

                        JOptionPane.showMessageDialog(
                                this,
                                "Circle path is out of robot reach.",
                                "Geometric Reach Error",
                                JOptionPane.ERROR_MESSAGE
                        );

                        return;
                    }
                }




                boolean angularOK;

                if (pathType.equals("Line")) {

                    angularOK =
                            checkLineAngularReach(
                                    A1, A2, A3,
                                    j1Min, j1Max,
                                    j2Min, j2Max,
                                    j3Min, j3Max,
                                    X1, Y1,
                                    X2, Y2
                            );

                    if (!angularOK) {

                        JOptionPane.showMessageDialog(
                                this,
                                "Robot cannot follow this line due to joint angle limits.",
                                "Joint Limit Reach Error",
                                JOptionPane.ERROR_MESSAGE
                        );

                        return;
                    }

                } else {

                    angularOK =
                            checkCircleAngularReach(
                                    A1, A2, A3,
                                    j1Min, j1Max,
                                    j2Min, j2Max,
                                    j3Min, j3Max,
                                    CX, CY, R
                            );

                    if (!angularOK) {

                        JOptionPane.showMessageDialog(
                                this,
                                "Robot cannot draw this circle due to joint angle limits.",
                                "Joint Limit Reach Error",
                                JOptionPane.ERROR_MESSAGE
                        );

                        return;
                    }
                }
            }



            boolean manualModeEnabled =
                    mode.equalsIgnoreCase("Manual");


            new RobotSimulatorFrame(
                    A1, A2, A3,
                    pathType,
                    X1, Y1, X2, Y2,
                    CX, CY, R,
                    j1Min, j1Max,
                    j2Min, j2Max,
                    j3Min, j3Max,
                    pressure,
                    manualModeEnabled
            );

            dispose();

        } catch (Exception ex) {

            JOptionPane.showMessageDialog(
                    this,
                    "Invalid Input!",
                    "Error",
                    JOptionPane.ERROR_MESSAGE
            );
        }
    }
}