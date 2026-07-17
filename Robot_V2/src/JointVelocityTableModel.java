import javax.swing.table.AbstractTableModel;

public class JointVelocityTableModel extends AbstractTableModel {

    RobotArm robot;

    String[] cols={"Joint","ThetaDot"};

    public JointVelocityTableModel(RobotArm r){
        robot=r;
    }

    public int getRowCount(){ return 3; }

    public int getColumnCount(){ return 2; }

    public String getColumnName(int c){ return cols[c]; }

    public Object getValueAt(int r,int c){

        if(r==0) return c==0?"Joint1":robot.thetaDot1;
        if(r==1) return c==0?"Joint2":robot.thetaDot2;
        if(r==2) return c==0?"Joint3":robot.thetaDot3;

        return "";
    }

    public void refresh(){
        fireTableDataChanged();
    }
}
