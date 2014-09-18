package com.ailab;

import com.ailab.tools.Path;

import java.lang.*;
import java.io.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.nio.file.FileSystems;
import java.nio.file.Files;

public class DrawPath extends Frame {

    public Point myPoint[];
    public static void drawPixel(Graphics g, int x, int y, int size, Paint color)
    {
        Graphics2D ga = (Graphics2D)g;
        Shape circle = new Ellipse2D.Float(x, y, size, size);
        ga.setPaint(color);
        ga.draw(circle);
        ga.setPaint(color);
        ga.fill(circle);
    }

    public void paint(Graphics g) {
        Graphics2D ga = (Graphics2D)g;
        for(int i = 0; i < myPoint.length; i++) {
            drawPixel(g, myPoint[i].x, myPoint[i].y, 3, Color.blue);
        }
    }



    public static void main(String args[])
    {

        DrawPath frame = new DrawPath();
        Path path = null;
        try {
            path = new Path(Files.newInputStream(FileSystems.getDefault().getPath(args[0])));
        } catch (IOException e) {
            e.printStackTrace();
        }
        frame.myPoint = new Point[path.size()];
        for(int i = 0; i < path.size(); i++) {
            System.out.println("X " + Math.round((float)((path.get(i).getPose().getPosition().getX()/10) * 400)));
            System.out.println("Y " + Math.round((float)((path.get(i).getPose().getPosition().getY()/10) * 400)));
            frame.myPoint[i] = new Point();
            frame.myPoint[i].x = 400 - Math.round((float)((path.get(i).getPose().getPosition().getY()/10) * 400));
            frame.myPoint[i].y = 400 - Math.round((float)((path.get(i).getPose().getPosition().getX()/10) * 400));
        }

        frame.addWindowListener(
                new WindowAdapter()
                {
                    public void windowClosing(WindowEvent we)
                    {
                        System.exit(0);
                    }
                }
        );

        frame.setSize(400, 400);
        frame.setVisible(true);
    }
}