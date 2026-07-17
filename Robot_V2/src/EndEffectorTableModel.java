import javax.swing.table.AbstractTableModel;

public class EndEffectorTableModel extends AbstractTableModel {

    RobotArm robot;

    String[] cols = {"Item", "X", "Y"};

    public EndEffectorTableModel(RobotArm r) {
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

        double[] pos = robot.endEffectorPosition();
        double[] vel = robot.endEffectorVelocity();

        double v = Math.sqrt(vel[0] * vel[0] + vel[1] * vel[1]);

        if (r == 0) return c == 0 ? "Position" : c == 1 ? pos[0] : pos[1];
        if (r == 1) return c == 0 ? "Velocity" : c == 1 ? vel[0] : vel[1];
        if (r == 2) return c == 0 ? "|V|" : c == 1 ? v : "";

        return "";
    }

    public void refresh() {
        fireTableDataChanged();
    }
}
