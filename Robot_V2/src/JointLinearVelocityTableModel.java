import javax.swing.table.AbstractTableModel;

public class JointLinearVelocityTableModel extends AbstractTableModel {

    RobotArm robot;

    String[] cols = {"Joint", "Vx", "Vy"};

    public JointLinearVelocityTableModel(RobotArm r) {
        robot = r;
    }

    public int getRowCount() {
        return 3;
    }

    public int getColumnCount() {
        return 3;
    }

    public String getColumnName(int c) {
        return cols[c];
    }

    public Object getValueAt(int r, int c) {

        double[] v1 = robot.joint1Velocity();
        double[] v2 = robot.joint2Velocity();
        double[] v3 = robot.joint3Velocity();

        if (r == 0) return c == 0 ? "Joint1" : c == 1 ? v1[0] : v1[1];
        if (r == 1) return c == 0 ? "Joint2" : c == 1 ? v2[0] : v2[1];
        if (r == 2) return c == 0 ? "Joint3" : c == 1 ? v3[0] : v3[1];

        return "";
    }

    public void refresh() {
        fireTableDataChanged();
    }
}
