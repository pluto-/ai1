package com.ailab.tools;

import java.lang.*;
import java.awt.*;
import java.awt.geom.*;
import java.util.ArrayList;

public class DrawPath extends Frame {

    public ArrayList<Point> bluePoints;
    public ArrayList<Point> redPoints;
    public ArrayList<Point> yellowPoints;
    public Point greenPoint;
    public void drawPixel(Graphics g, int x, int y, int size, Paint color)
    {
        Graphics2D ga = (Graphics2D)g;
        Shape circle = new Ellipse2D.Float(x, y, size, size);
        ga.setPaint(color);
        ga.draw(circle);
        ga.setPaint(color);
        ga.fill(circle);
    }
    /*
    public void drawCar(Graphics g)
    {
        Graphics2D ga = (Graphics2D)g;
        Shape circle = new Square();
        ga.setPaint(color);
        ga.draw(circle);
        ga.setPaint(color);
        ga.fill(circle);
    }*/

    public void paint(Graphics g) {
        Graphics2D ga = (Graphics2D)g;
        for(int i = 0; i < bluePoints.size(); i++) {
            drawPixel(g, bluePoints.get(i).x, bluePoints.get(i).y, 3, Color.blue);
        }
        for(int i = 0; i < redPoints.size(); i++) {
            drawPixel(g, redPoints.get(i).x, redPoints.get(i).y, 3, Color.red);
        }
        for(int i = 0; i < yellowPoints.size(); i++) {
            drawPixel(g, yellowPoints.get(i).x, yellowPoints.get(i).y, 3, Color.yellow);
        }
        drawPixel(g, greenPoint.x, greenPoint.y, 7, Color.green);

    }

    public void addRedPoint(double x, double y) {
        Point redPoint = new Point();
        redPoint.x = 400 - Math.round((float)((y/10) * 400));
        redPoint.y = 400 - Math.round((float)((x/10) * 400));
        redPoints.add(redPoint);
    }

    public void addYellowPoint(double x, double y) {
        Point yellowPoint = new Point();
        yellowPoint.x = 400 - Math.round((float)((y/10) * 400));
        yellowPoint.y = 400 - Math.round((float)((x/10) * 400));
        yellowPoints.add(yellowPoint);
    }
    public void setGreenPoint(double x, double y) {
        greenPoint = new Point();
        greenPoint.x = 400 - Math.round((float)((y/10) * 400));
        greenPoint.y = 400 - Math.round((float)((x/10) * 400));
    }

    public DrawPath(Path path) {
        super("Path");

        this.bluePoints = new ArrayList<Point>();
        this.redPoints = new ArrayList<Point>();
        this.yellowPoints = new ArrayList<Point>();
        for(int i = 0; i < path.size(); i++) {
            //System.out.println("X " + Math.round((float)((path.get(i).getPose().getPosition().getX()/10) * 400)));
            //System.out.println("Y " + Math.round((float)((path.get(i).getPose().getPosition().getY()/10) * 400)));
            Point myPoint = new Point();
            myPoint.x = 400 - Math.round((float)((path.get(i).getPose().getPosition().getY()/10) * 400));
            myPoint.y = 400 - Math.round((float)((path.get(i).getPose().getPosition().getX()/10) * 400));
            bluePoints.add(myPoint);
        }


        this.setSize(400, 400);
        this.setVisible(true);
    }


}