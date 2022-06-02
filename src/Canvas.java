import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.BufferedImageOp;
import java.awt.image.ImageObserver;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import javax.imageio.ImageIO;
import javax.swing.*;


public class Canvas extends JPanel {

    ArrayList<Vehicle> 			allVehicles;
    double pix;
    ImageIcon imgIcon = null;
    Canvas(ArrayList<Vehicle> allVehicles, double pix){
        this.allVehicles = allVehicles;
        this.pix         = pix;
        this.setBackground(Color.decode("#466D1D"));
        setSize(5000,5000);
        //img1 = Toolkit.getDefaultToolkit().getImage("dog.gif");
        imgIcon = new ImageIcon("C:\\Users\\Lucas Toulon\\Documents\\Projekte\\HFT Stuttgart\\MSc\\Intelligente Systeme\\Simulation\\src\\doge3.png");
        Image image = imgIcon.getImage(); // transform it
        Image newimg = image.getScaledInstance(36, 36,  java.awt.Image.SCALE_SMOOTH); // scale it the smooth way
        imgIcon = new ImageIcon(newimg);  // transform it back
    }


    public Polygon kfzInPolygon(Vehicle fz){

        Polygon   q = new Polygon();
        int    l    = (int)(fz.FZL/pix);
        int    b    = (int)(fz.FZB/pix);
        int    x    = (int)(fz.pos[0]/pix);
        int    y    = (int)(fz.pos[1]/pix);
        int    dia  = (int)(Math.sqrt(Math.pow(l/2, 2)+Math.pow(b/2, 2)));
        double    t = fz.winkel(fz.vel);
        double phi1 = Math.atan(fz.FZB/fz.FZL);
        double phi2 = Math.PI-phi1;
        double phi3 = Math.PI+phi1;
        double phi4 = 2*Math.PI-phi1;
        int      x1 = (int)(x+(dia*Math.cos(t+  phi1)));
        int      y1 = (int)(y+(dia*Math.sin(t+  phi1)));
        int      x2 = (int)(x+(dia*Math.cos(t+  phi2)));
        int      y2 = (int)(y+(dia*Math.sin(t+  phi2)));
        int      x3 = (int)(x+(dia*Math.cos(t+  phi3)));
        int      y3 = (int)(y+(dia*Math.sin(t+  phi3)));
        int      x4 = (int)(x+(dia*Math.cos(t+  phi4)));
        int      y4 = (int)(y+(dia*Math.sin(t+  phi4)));
        q.addPoint(x1, y1);
        q.addPoint(x2, y2);
        q.addPoint(x3, y3);
        q.addPoint(x4, y4);
        return q;
    }

    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;

        for(int i=0;i<allVehicles.size();i++){
            Vehicle fz = allVehicles.get(i);
            Polygon q = kfzInPolygon(fz);

            //Der Hund kriegt ein Bild verpasst
            if(fz.type==1){
               if(imgIcon != null){
                   imgIcon.paintIcon(this, g, (int)(fz.pos[0]/pix)-18, (int)(fz.pos[1]/pix)-18); //zentriert
               }else{
                   //Bei Bildproblemen lieber das Rechteck nehmen
                   g2d.setColor(Color.DARK_GRAY);
                   g2d.fillPolygon(q);
               }
            }else{
                g2d.setColor(Color.lightGray);
                g2d.fillPolygon(q);
            }

            int    x  = (int)(fz.pos[0]/pix);
            int    y  = (int)(fz.pos[1]/pix);

            //Falls Bild nicht vorhanden, dann zwei Kreise für die verschiedenen Radien
            if(fz.type==1){
                //
                int seite = (int)(fz.rad_sepSchafzuSchaeferhund/pix);
                g2d.drawOval(x-seite, y-seite, 2*seite, 2*seite);
                seite = (int)(fz.rad_ErkennungSchaeferhundzuSchaf /pix);
                g2d.drawOval(x-seite, y-seite, 2*seite, 2*seite);
            }
        }

        //Jetzt noch der Zaun
        g2d.drawLine(500, 0, 500, 350);
        g2d.drawLine(500, 450, 500, 800);
    }
}
