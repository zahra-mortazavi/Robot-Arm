import javax.swing.*;
import javax.swing.table.DefaultTableCellRenderer;
import java.awt.*;
import java.text.DecimalFormat;

public class ScientificRenderer extends DefaultTableCellRenderer {

    DecimalFormat format = new DecimalFormat("0.000E0");

    public Component getTableCellRendererComponent(
            JTable table,Object value,boolean isSelected,
            boolean hasFocus,int row,int column){

        super.getTableCellRendererComponent(
                table,value,isSelected,hasFocus,row,column);

        if(value instanceof Number){
            setText(format.format(value));
        }

        setHorizontalAlignment(SwingConstants.CENTER);

        return this;
    }
}
