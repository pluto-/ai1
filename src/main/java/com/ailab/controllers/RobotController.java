package com.ailab.controllers;

import com.ailab.tools.*;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;

/**
 * Created by Jonas on 2014-09-15.
 */
public class RobotController implements Runnable {

    private final static double LOOK_AHEAD = 1;
    private final static double PROPORTIONAL_GAIN = 1;

    private Robot robot;
    private Path path;
    private int indexOfLastClosestNode = 0;
    private int indexOfLastTargetNode = 0;
    static final Logger logger = LogManager.getLogger(RobotController.class.getName());
    int delay = 0;
    DrawPath drawPath;
    VFHPlus vfhPlus;

    public RobotController(Robot robot, String pathFile) throws IOException {
        this.robot = robot;

        path = new Path(Files.newInputStream(FileSystems.getDefault().getPath(pathFile)));
        drawPath = new DrawPath(path);
        LaserPropertiesResponse laserPropertiesResponse = new LaserPropertiesResponse();
        robot.getResponse(laserPropertiesResponse);
        vfhPlus = new VFHPlus(laserPropertiesResponse.getStartAngle(), laserPropertiesResponse.getEndAngle(), laserPropertiesResponse.getAngleIncrement());
        Thread thread = new Thread(this);
        thread.start();
    }

    public void start() throws Exception {

        double curvature;
        double speedAndAngularSpeed[];


        try {
            while (true) {
                if (indexOfLastTargetNode == path.size() - 1) {

                    LocalizationResponse localizationResponse = new LocalizationResponse();
                    robot.getResponse(localizationResponse);
                    double distance = new Position(localizationResponse.getPosition()).getDistanceTo(path.get(path.size() - 1).getPosition());
                    if (distance < 0.2) {
                        logger.error("distance to goal: " + distance);
                        robot.drive(0, 0);
                        break;
                    }
                }
                LocalizationResponse localizationResponse = new LocalizationResponse();
                robot.getResponse(localizationResponse);
                //System.out.println("X = " + localizationResponse.getPosition()[0] + " Y = " + localizationResponse.getPosition()[1]);
                //drawPath.addRedPoint(localizationResponse.getPosition()[0], localizationResponse.getPosition()[1]);
                curvature = pursue(path);
                speedAndAngularSpeed = setSpeedAndAngularSpeed(curvature);

                //speed = (angularSpeed == 0 ? 1 : Math.abs(1.0 / angularSpeed));
                robot.drive(speedAndAngularSpeed[0], speedAndAngularSpeed[1]);
                /*try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {

                }*/
            }
        } catch (Exception e) {
            robot.drive(0,0);
            e.printStackTrace();
        }
    }

    private double[] setSpeedAndAngularSpeed(double curvature) {
        // 0 <= speed <= 1.
        double speed;
        double angularSpeed;

        if(Math.abs(curvature) > 2) {
            speed = 0.1;
            if(curvature < 0)
                angularSpeed = -1;
            else
                angularSpeed = 1;
        } else {
            speed = Math.abs(1 - Math.abs(curvature) / 2);
            angularSpeed = curvature;
        }
        return new double[] {speed, angularSpeed};
    }

    public double pursue(Path path) throws IOException {
        // 1. Determine vehicle position.
        LocalizationResponse localizationResponse = new LocalizationResponse();
        localizationResponse = (LocalizationResponse)robot.getResponse(localizationResponse);
        Position currentPosition = new Position(localizationResponse.getPosition());
        double heading = localizationResponse.getHeadingAngle();
        // 2. Find the point on the path closes to the vehicle, 3. Find the carrot point.
        Position carrotPosition = calcCarrotPosition(LOOK_AHEAD, currentPosition);
        drawPath.setGreenPoint(carrotPosition.getX(), carrotPosition.getY());

        // 4. Transform the carrot point and the vehicle location to the vehicle coordinates.
        double distanceToCarrotPoint = currentPosition.getDistanceTo(carrotPosition);
        double directionToCarrotPoint = currentPosition.getBearingTo(carrotPosition);



        LaserEchoesResponse laserEchoesResponse  = new LaserEchoesResponse();
        robot.getResponse(laserEchoesResponse);
        double echoes[] = laserEchoesResponse.getEchoes();

        boolean obstacleNearby = false;
        for(int i = 0; i < echoes.length; i++) {
            if(echoes[i] <= distanceToCarrotPoint) {
                obstacleNearby = true;
                break;
            }
        }

        if(obstacleNearby) {
            double ftcAngle = followTheCarrot(heading, directionToCarrotPoint);
            Double vfhAngle = vfhPlus.calculateSteeringDirection(distanceToCarrotPoint, echoes, ftcAngle);
            if(vfhAngle == null) {
                logger.error("No valley found. Stopping vehicle...");
                return 1;
            }
            if(ftcAngle != vfhAngle) {
                logger.error("USING VFH+ ANGLE: " + vfhAngle + "         VFH - FTC: " + (vfhAngle - ftcAngle));
                if(vfhAngle > 0) {
                    drawPath.addRedPoint(localizationResponse.getPosition()[0], localizationResponse.getPosition()[1]);

                } else {

                    drawPath.addYellowPoint(localizationResponse.getPosition()[0], localizationResponse.getPosition()[1]);
                }

                return vfhAngle;
            }
            vfhPlus.resetPreviousTargetSector();

        }
        vfhPlus.resetPreviousTargetSector();
        return purePursuit(heading, currentPosition, carrotPosition, distanceToCarrotPoint);



        //double alpha = - heading + currentPosition.getBearingTo(carrotPosition);
        //Double vfhAngle = vfhPlus.calculateSteeringDirection(distanceToCarrotPoint,laserEchoesResponse.getEchoes(), ftcAngle);
        //double purePursuit = purePursuit(heading, currentPosition, carrotPosition, distanceToCarrotPoint);
        //System.out.println("VFS STEERING DIRECTION = " + vfhAngle);
        //System.out.println("FOLLOW CARROT DIRECTION = " + ftcAngle);

        //return followTheCarrot(heading, directionToCarrotPoint);


        //Map<Integer, Double> stuff = vfhPlus.buildPolarHistogram(1.0, laserEchoesResponse.getEchoes(), laserPropertiesResponse.getStartAngle(), 0);

        /*if(delay++ > 500) {
            logger.error("VFH+ angle: " + (vfhAngle == null ? "NULL " : vfhAngle) + " PurePursuit angle: " + alpha);
            logger.error("VFH+ curvature: " + (vfhAngle == null ? "NULL " : vfhAngle) + " PurePursuit curvature: " + (2 * distanceToCarrotPoint * Math.cos(alpha) / (distanceToCarrotPoint * distanceToCarrotPoint)));
            delay = 0;
        }
        if (vfhAngle != null && vfhAngle != alpha) {
            double curvature = vfhAngle*0.8;

            return curvature;

        } else {
            double deltaX = distanceToCarrotPoint * Math.cos(alpha);
            double curvature = (2*deltaX / (distanceToCarrotPoint*distanceToCarrotPoint));
            return curvature;
            // 5. Calculate the curvature of the circular arc.
            // System.out.println("VEHICLE POS X: " + currentPosition.getX() + " Y: " + currentPosition.getY() + " curvature: " + curvature);

            // 6. Determine the steering angle.
        }*/


        /*
        if(delay++ > 500) {
            logger.error("Carrot Point: " + carrotPosition + " alpha: " + alpha + " heading: " + heading);
            logger.error("alpha - heading: " + (alpha - heading) + " alpha + heading: " + (alpha + heading));
            delay = 0;
        }*/
        //return curvature;
    }
    private double purePursuit(double heading, Position currentPosition, Position carrotPosition, double distanceToCarrotPoint) {
        //double alpha = heading + Math.atan2(carrotPosition.getX() - currentPosition.getX(), carrotPosition.getY() - currentPosition.getY());
        //double alpha = heading + Math.atan2(carrotPosition.getY() - currentPosition.getY(), carrotPosition.getX() - currentPosition.getX());
        double alpha = - heading + currentPosition.getBearingTo(carrotPosition);

        //System.out.println("HEADING = " + heading);
        //System.out.println("ORIENTATION = " + currentPosition.getBearingTo(carrotPosition));
        //System.out.println("DIFFERENCE = " + (heading - Math.atan2(carrotPosition.getX() - currentPosition.getX(), carrotPosition.getY() - currentPosition.getY())));

        double deltaX = distanceToCarrotPoint * Math.cos(alpha);
        double curvature = ((-2)*deltaX / (distanceToCarrotPoint*distanceToCarrotPoint));
        return curvature;
    }

    private double followTheCarrot(double heading, double directionToCarrotPoint) {
        double e = - heading - Math.PI/2 + directionToCarrotPoint;
        double e_prime = e * PROPORTIONAL_GAIN;
        double e_0;
        if (e_prime > Math.PI)
            e_0 = e - 2*Math.PI;
        else if (e_prime < -Math.PI)
            e_0 = e_prime + 2*Math.PI;
        else
            e_0 = e_prime;

        return e_0;
    }

    private Position calcCarrotPosition(double lookAhead, Position vehicle_pos) {
        double shortestDistance = Integer.MAX_VALUE;
        DistanceData distance;
        int shortestNodeNumber = -1;
        int type = -1;
        for(int i = indexOfLastClosestNode; i < path.size() && path.get(indexOfLastClosestNode).getPosition().getDistanceTo(path.get(i).getPosition()) < 2*lookAhead; i++) {
            distance = calcDistance(path.get(i).getPosition(), path.get(i + 1).getPosition(), vehicle_pos);
            if(shortestDistance > distance.getDistance()) {
                shortestDistance = distance.getDistance();
                shortestNodeNumber = indexOfLastClosestNode = i;
                type = distance.getType();
            }
        }

        int indexOfStartNode = shortestNodeNumber;
        if(type == DistanceData.P1) {
            indexOfStartNode++;
        } else if(type == DistanceData.SEGMENT) {
            Position p0 = path.get(shortestNodeNumber).getPose().getPosition();
            Position p1 = path.get(shortestNodeNumber + 1).getPose().getPosition();
            Position position = Util.getClosestPointOnSegment(p0, p1, vehicle_pos);
            lookAhead += path.get(shortestNodeNumber).getPosition().getDistanceTo(position);
        }
        return getCarrotPosition(path.get(indexOfStartNode), path.get(indexOfStartNode + 1), path, lookAhead);
    }

    public Position getCarrotPosition(PathNode current, PathNode next, Path path, double lookAhead) {
        indexOfLastTargetNode = current.getIndex();
        if(lookAhead == 0  || path.isLastNode(current)) {
            return current.getPosition();
        } else if(lookAhead >= current.getPosition().getDistanceTo(next.getPosition())) {
            double new_lookAhead = lookAhead - current.getPosition().getDistanceTo(next.getPosition());
            return getCarrotPosition(next, path.get(next.getIndex() + 1), path, new_lookAhead);
        } else {
            Position carrotPosition = new Position();
            double k = lookAhead / current.getPosition().getDistanceTo(next.getPosition());
            double x = current.getPosition().getX() + (next.getPosition().getX() - current.getPosition().getX()) * k;
            double y = current.getPosition().getY() + (next.getPosition().getY() - current.getPosition().getY()) * k;

            carrotPosition.setX(x);
            carrotPosition.setY(y);

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

    @Override
    public void run() {
        while(true) {
            drawPath.repaint();
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
