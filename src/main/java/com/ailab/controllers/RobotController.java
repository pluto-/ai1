package com.ailab.controllers;

import com.ailab.tools.*;

import java.awt.*;
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

    Robot robot;
    Path path;

    public RobotController(Robot robot, String pathFile) throws IOException {
        this.robot = robot;

        path = new Path(Files.newInputStream(FileSystems.getDefault().getPath(pathFile)));
    }

    public void start() throws Exception {

        robot.drive(0.1, pursue(path));
    }

    public double pursue(Path path) throws Exception {
        double angle = 0.0;

        // 1. Determine vehicle position.
        Position vehicle_pos = new Position(((LocalizationResponse)robot.getResponse(new LocalizationResponse())).getPosition());
        System.out.println("VEHICLE POS X: " + vehicle_pos.getX() + " Y: " + vehicle_pos.getY());

        // 2. Find the point on the path closes to the vehicle.
        PathNode nextPosition = path.getNext();
        System.out.println("NEXT POS " + nextPosition.toString());

        // 3. Find the carrot point.
        double shortestDistance = 10000.0;
        DistanceData distance;
        int shortestNodeNumber = -1;
        int type = -1;
        for(int i = 0; i < path.size() - 1; i++) {
            distance = calcDistance(path.get(i).getPose().getPosition(), path.get(i + 1).getPose().getPosition(), vehicle_pos);
            if(shortestDistance > distance.getDistance()) {
                shortestDistance = distance.getDistance();
                shortestNodeNumber = i;
            }
            System.out.println("DISTANCE = " + distance + " NODE NUMBER = " + i);
        }

        PathNode nodeToAimFor;
        if(type == DistanceData.P0) {
            nodeToAimFor = path.get(shortestNodeNumber);
        } else if(type == DistanceData.P1) {
            nodeToAimFor = path.get(shortestNodeNumber + 1);
        } else if(type == DistanceData.SEGMENT) {
            nodeToAimFor = new PathNode();
            Pose pose = new Pose();

            Position p0 = path.get(shortestNodeNumber).getPose().getPosition();
            Position p1 = path.get(shortestNodeNumber + 1).getPose().getPosition();
            Position position = Util.getClosestPointOnSegment(p0, p1, vehicle_pos);

            pose.setPosition(position);
            nodeToAimFor.setPose(pose);
        }

        System.out.println("SHORTEST DISTANCE = " + shortestDistance + " NODE NUMBER = " + shortestNodeNumber);

        double lookAhead = 10.0;

        double vehicle_angle = robot.getBearingAngle(new LocalizationResponse());
        System.out.println("VEHICLE ANGLE = " + vehicle_angle);

        // 4. Transform the carrot point and the vehicle location to the vehicle coordinates.
        // 5. Calculate the curvature of the circular arc.
        // 6. Determine the steering angle.
        return angle;
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
