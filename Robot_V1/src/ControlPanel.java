import javax.swing.*;
import javax.swing.border.TitledBorder;
import javax.swing.table.AbstractTableModel;
import java.awt.*;

public class ControlPanel extends JPanel {

    private RobotArm robot;

    JointAngleTableModel angleModel;
    JointVelocityTableModel velModel;
    JointLinearVelocityTableModel linVelModel;
    EndEffectorTableModel eeModel;

    public ControlPanel(boolean isManual) {
        setLayout(new BorderLayout());
    }

    public void attachRobot(RobotArm robot) {
        this.robot = robot;

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

        Timer timer = new Timer(100, e -> {
            angleModel.refresh();
            velModel.refresh();
            linVelModel.refresh();
            eeModel.refresh();
        });
        timer.start();
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
}
