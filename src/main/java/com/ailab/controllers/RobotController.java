package com.ailab.controllers;

import com.ailab.tools.*;

import java.awt.*;
import java.awt.geom.Arc2D;
import java.io.BufferedInputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.util.Vector;

/**
 * Created by Jonas on 2014-09-15.
 */
public class RobotController {

    private final static double LOOKAHEAD = 1.0;

    Robot robot;
    Path path;

    public RobotController(Robot robot, String pathFile) throws IOException {
        this.robot = robot;

        path = new Path(Files.newInputStream(FileSystems.getDefault().getPath(pathFile)));
    }

    public void start() throws Exception {

        double curvature = 0.0, speed;

        try {
            while (true) {
                curvature = pursue(path);
                speed = Math.abs(1.0 / curvature);
                robot.drive(speed, speed * (pursue(path)));
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {

                }
            }
        } catch (Exception e) {
            robot.drive(0,0);
        }
    }

    public double pursue(Path path) throws Exception {
        // 1. Determine vehicle position.
        LocalizationResponse localizationResponse = new LocalizationResponse();
        localizationResponse = (LocalizationResponse)robot.getResponse(localizationResponse);
        Position currentPosition = new Position(localizationResponse.getPosition());
        double heading = localizationResponse.getHeadingAngle();
        // 2. Find the point on the path closes to the vehicle, 3. Find the carrot point.
        Position carrotPosition = calcCarrotPosition(LOOKAHEAD, currentPosition);

        // 4. Transform the carrot point and the vehicle location to the vehicle coordinates.
        double distance = currentPosition.getDistanceTo(carrotPosition);
        double alpha = heading + Math.atan2(carrotPosition.getX() - currentPosition.getX(), carrotPosition.getY() - currentPosition.getY());
        double deltaX = distance * Math.cos(alpha);
        double curvature = (2*deltaX / (distance*distance));
        // 5. Calculate the curvature of the circular arc.
        System.out.println("VEHICLE POS X: " + currentPosition.getX() + " Y: " + currentPosition.getY() + " curvature: " + curvature);

        // 6. Determine the steering angle.
        return curvature;
    }

    private Position calcCarrotPosition(double lookAhead, Position vehicle_pos) {
        double shortestDistance = 10000.0;
        DistanceData distance;
        int shortestNodeNumber = -1;
        int type = -1;
        for(int i = 0; i < path.size() - 1; i++) {
            distance = calcDistance(path.get(i).getPose().getPosition(), path.get(i + 1).getPose().getPosition(), vehicle_pos);
            if(shortestDistance > distance.getDistance()) {
                shortestDistance = distance.getDistance();
                shortestNodeNumber = i;
                type = distance.getType();
            }
            //System.out.println("DISTANCE = " + distance + " NODE NUMBER = " + i);
        }

        PathNode closestPointOnPath = null;
        PathNode nextNode = null;
        if(type == DistanceData.P0) {
            closestPointOnPath = path.get(shortestNodeNumber);
            nextNode = path.get(shortestNodeNumber + 1);
        } else if(type == DistanceData.P1) {
            closestPointOnPath = path.get(shortestNodeNumber + 1);
            nextNode = path.get(shortestNodeNumber + 2);
        } else if(type == DistanceData.SEGMENT) {
            closestPointOnPath = new PathNode();
            Pose pose = new Pose();

            Position p0 = path.get(shortestNodeNumber).getPose().getPosition();
            Position p1 = path.get(shortestNodeNumber + 1).getPose().getPosition();
            Position position = Util.getClosestPointOnSegment(p0, p1, vehicle_pos);

            pose.setPosition(position);
            closestPointOnPath.setPose(pose);
            nextNode = path.get(shortestNodeNumber + 1);
        }
        System.out.println(closestPointOnPath);
        return getCarrotPosition(closestPointOnPath.getPose().getPosition(), nextNode, path, lookAhead);
    }

    public static Position getCarrotPosition(Position current, PathNode next, Path path, double lookAhead) {

        if(lookAhead <= 0  || path.size() < next.getIndex()) {
            return current;
        } else if(lookAhead >= current.getDistanceTo(next.getPose().getPosition()) && path.size() > next.getIndex()) {

            Position new_current = next.getPose().getPosition();
            PathNode new_next = path.get(next.getIndex() + 1);
            double new_lookAhead = lookAhead - current.getDistanceTo(next.getPose().getPosition());
            //System.out.print("Look Ahead is longer beyond X = " + next.getPose().getPosition().getX() + " Y = " + next.getPose().getPosition().getY());
            //System.out.println(". Look Ahead left is = " + new_lookAhead);
            return getCarrotPosition(new_current, new_next, path, new_lookAhead);
        } else {
            Position carrotPosition = new Position();
            double k = lookAhead / current.getDistanceTo(next.getPose().getPosition());
            double x = (next.getPose().getPosition().getX() - current.getX()) * k;
            double y = (next.getPose().getPosition().getY() - current.getY()) * k;
            carrotPosition.setX(current.getX() + x);
            carrotPosition.setY(current.getY() + y);
            return carrotPosition;
        }
    }

    // From algorithm in https://www8.cs.umu.se/kurser/5DV121/HT14/utdelat/Ringdahl%202003%20Master%20thesis.pdf
    // page 12 - 14.
    private static DistanceData calcDistance(Position p0, Position p1, Position p) {
        double v[] = new double[2];
        v[0] = p1.getX() - p0.getX();
        v[1] = p1.getY() - p0.getY();

        double w[] = new double[2];
        w[0] = p.getX() - p0.getX();
        w[1] = p.getY() - p0.getY();

        double c1 = dotProduct(w, v);
        double c2 = dotProduct(v, v);

        if(c1 <= 0) {
            // d(P, P0)
            return new DistanceData(p.getDistanceTo(p0), DistanceData.P0);
        }

        if(c2 <= c1) {
            // d(P, P1)
            return new DistanceData(p.getDistanceTo(p1), DistanceData.P1);
        }

        double b = c1 / c2;
        double pb[] = new double[2];
        pb[0] = p0.getX() + b*v[0];
        pb[1] = p0.getY() + b*v[1];

        // d(P, Pb)
        return new DistanceData(p.getDistanceTo(new Position(pb)), DistanceData.SEGMENT);
    }

    private static double dotProduct(double[] a, double[] b){
        if(a.length != b.length){
            throw new IllegalArgumentException("The dimensions have to be equal!");
        }
        double sum = 0;
        for(int i = 0; i < a.length; i++){
            sum += a[i] * b[i];
        }
        return sum;
    }
}
