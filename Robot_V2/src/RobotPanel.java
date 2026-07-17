import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class RobotPanel extends JPanel {

    RobotArm robot;
    boolean showGrid = true;
    boolean showWorkspace = true;
    boolean showTrail = true;
    boolean manualMode = false;
    String pressure = "Medium";



    private BufferedImage link1Img, link2Img, link3Img, joint1Img, joint2Img, joint3Img, earthImg;

    ArrayList<Point2D.Double> trail = new ArrayList<>();
    double scale = 80;

    Path targetPath;
    boolean drawTargetPath = true;

    public RobotPanel(RobotArm r) {
        robot = r;
        setPreferredSize(new Dimension(600, 600));
        loadImages();
    }

    private void loadImages() {
        try {
            link1Img = ImageIO.read(new File("link1.png"));
            link2Img = ImageIO.read(new File("link2.png"));
            link3Img = ImageIO.read(new File("link3.png"));
            joint1Img = ImageIO.read(new File("joint1.png"));
            joint2Img = ImageIO.read(new File("joint2.png"));
            joint3Img = ImageIO.read(new File("joint2.png"));
            earthImg = ImageIO.read(new File("earth.png"));

        } catch (IOException e) {
            System.err.println("Error loading images: " + e.getMessage());
        }
    }
    private void drawLinkImage(Graphics2D g2, BufferedImage img,
                               double x1, double y1, double x2, double y2) {

        if (img == null) return;

        double dx = x2 - x1;
        double dy = y2 - y1;

        dy = dy;

        double length = Math.sqrt(dx*dx + dy*dy);

        double angle = Math.atan2(dy, dx);

        AffineTransform old = g2.getTransform();

        g2.translate(x1, y1);


        g2.rotate(angle - Math.PI/2);
        int w  = (img == link3Img) ? 50 : img.getWidth();;


        g2.drawImage(img, -w/2, 0, w, (int)length, null);

        g2.setTransform(old);
    }

    private void drawJointImage(Graphics2D g2, BufferedImage img, double x, double y) {
        if (img == null) return;

        int size = (img == joint1Img) ? 90 : 50;
        g2.drawImage(img, (int) x - size / 2, (int) y - size / 2, size, size, null);
    }


    public void setPressure(String p) {
        this.pressure = p;
    }


    public void setTargetPath(Path p) { this.targetPath = p; }

    public void setManualMode(boolean v) {
        manualMode = v;
        if (v) {
            drawTargetPath = false;
            trail.clear();
        }
    }

    public void setShowGrid(boolean v) { showGrid = v; }
    public void setShowWorkspace(boolean v) { showWorkspace = v; }
    public void setShowTrail(boolean v) { showTrail = v; }

    public void updateTrail() {
        double[] p = robot.forwardKinematics();
        trail.add(new Point2D.Double(p[6], p[7]));
        if (trail.size() > 2000) trail.remove(0);
    }

    void drawArrow(Graphics2D g2, int x1, int y1, int x2, int y2) {
        g2.drawLine(x1, y1, x2, y2);
        double angle = Math.atan2(y2 - y1, x2 - x1);
        int arrowSize = 10;
        int xA = (int)(x2 - arrowSize * Math.cos(angle - Math.PI / 6));
        int yA = (int)(y2 - arrowSize * Math.sin(angle - Math.PI / 6));
        int xB = (int)(x2 - arrowSize * Math.cos(angle + Math.PI / 6));
        int yB = (int)(y2 - arrowSize * Math.sin(angle + Math.PI / 6));
        g2.drawLine(x2, y2, xA, yA);
        g2.drawLine(x2, y2, xB, yB);
    }

    double normalizeDeg(double a) {
        a = a % 360;
        if (a > 180) a -= 360;
        if (a < -180) a += 360;
        return a;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int cx = getWidth() / 2;
        int cy = getHeight() / 2;
        int w = getWidth();
        int h = getHeight();
    //GRID
        if (showGrid) {
            int gridStep = (int) scale;
            g2.setColor(new Color(230, 230, 230));
            for (int x = cx % gridStep; x < w; x += gridStep) g2.drawLine(x, 0, x, h);
            for (int y = cy % gridStep; y < h; y += gridStep) g2.drawLine(0, y, w, y);
        }

        //WORKSPACE
        if (showWorkspace) {
            double Rmax = robot.A1 + robot.A2 + robot.A3;
            int rMaxPix = (int)(Rmax * scale);
            g2.setColor(new Color(0,150,255,40));
            g2.drawOval(cx - rMaxPix, cy - rMaxPix, 2*rMaxPix, 2*rMaxPix);

            double Rmin = Math.abs(robot.A1 - robot.A2 - robot.A3);
            int rMinPix = (int)(Rmin * scale);
            g2.drawOval(cx - rMinPix, cy - rMinPix, 2*rMinPix, 2*rMinPix);
        }

        //AXES
        g2.setColor(Color.gray);
        g2.setStroke(new BasicStroke(1));
        g2.drawLine(0, cy, w, cy);
        g2.drawLine(cx, 0, cx, h);
        drawArrow(g2, cx, cy, w, cy);
        g2.drawString("X", w - 15, cy - 6);
        drawArrow(g2, cx, cy, cx, 0);
        g2.drawString("Y", cx + 6, 15);

    //TARGET PATH
        if (!manualMode && drawTargetPath && targetPath != null) {
            g2.setColor(new Color(255,165,0));
            int N = 300;
            double step = 1.0 / N;
            double s = 0;
            for (int i = 0; i < N; i++) {
                double[] p1 = targetPath.getPoint(s);
                double[] p2 = targetPath.getPoint(s + step);
                g2.drawLine((int)(cx + p1[0]*scale), (int)(cy - p1[1]*scale),
                        (int)(cx + p2[0]*scale), (int)(cy - p2[1]*scale));
                s += step;
            }
        }

        //FORWARD KINEMATICS
        double[] p = robot.forwardKinematics();

        int x1 = (int)(cx + p[2]*scale);
        int y1 = (int)(cy - p[3]*scale);
        int x2 = (int)(cx + p[4]*scale);
        int y2 = (int)(cy - p[5]*scale);
        int x3 = (int)(cx + p[6]*scale);
        int y3 = (int)(cy - p[7]*scale);

        double a1 = robot.theta1;
        double a2 = robot.theta1 + robot.theta2;
        double a3 = robot.theta1 + robot.theta2 + robot.theta3;

        if (showTrail) {

            if (pressure.equalsIgnoreCase("High"))
                g2.setColor(new Color(180,0,255));   // بنفش
            else if (pressure.equalsIgnoreCase("Medium"))
                g2.setColor(Color.red);              // قرمز
            else
                g2.setColor(Color.blue);
            for (int i = 1; i < trail.size(); i++) {
                int xa = (int)(cx + trail.get(i-1).x * scale);
                int ya = (int)(cy - trail.get(i-1).y * scale);
                int xb = (int)(cx + trail.get(i).x * scale);
                int yb = (int)(cy - trail.get(i).y * scale);
                g2.drawLine(xa, ya, xb, yb);
            }
        }

    // LINKS AND
        if (earthImg != null) {
            int wE = earthImg.getWidth();
            int hE = earthImg.getHeight();

            g2.drawImage(earthImg, cx - wE/2, cy+43, null);}
        drawLinkImage(g2, link3Img, x2, y2, x3, y3);

        drawLinkImage(g2, link2Img, x1, y1, x2, y2);

        drawLinkImage(g2, link1Img, cx, cy, x1, y1);
        drawJointImage(g2, joint3Img, x1, y1);
        drawJointImage(g2, joint2Img, x2, y2);
        drawJointImage(g2, joint1Img, cx, cy);



        //END EFFECTOR
        g2.setColor(Color.red);
        g2.fillOval(x3 - 5, y3 - 5, 10, 10);
        g2.drawString(String.format("(%.2f , %.2f)", p[6], p[7]), x3 + 10, y3 - 10);

        //JOINT ANGLES
        g2.setColor(Color.black);
        g2.drawString(String.format("θ1=%.1f°", normalizeDeg(Math.toDegrees(robot.theta1))), cx + 12, cy + 15);
        g2.drawString(String.format("θ2=%.1f°", normalizeDeg(Math.toDegrees(robot.theta2))), x1 + 12, y1 + 15);
        g2.drawString(String.format("θ3=%.1f°", normalizeDeg(Math.toDegrees(robot.theta3))), x2 + 12, y2 + 15);
    }
}
