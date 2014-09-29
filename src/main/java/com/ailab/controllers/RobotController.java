package com.ailab.controllers;

import com.ailab.tools.*;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;

/**
 * Contains all the methods for calculating the path tracking algorithms, determine the speed and angular speed and
 * methods for determining carrot position and distance. ALso contains the constants LOOK_AHEAD and PROPORTIONAL_GAIN
 * which are used by the algorithms.
 *
 * Created by Jonas on 2014-09-15.
 */
public class RobotController {

    private static boolean running = true;

    private static double LOOK_AHEAD = 2;
    private final static double PROPORTIONAL_GAIN = 1;

    private Robot robot;
    private Path path;
    private int indexOfLastClosestNode = 0;
    private int indexOfLastTargetNode = 0;
    static final Logger logger = LogManager.getLogger(RobotController.class.getName());
    int delay = 0;
    DrawPath drawPath;
    VFHPlus vfhPlus;

    /**
     * Constructor which reads the path from the JSON file and the properties of the laser sensor.
     * @param robot the robot.
     * @param pathFile path to the JSON file containing the path of the robot.
     * @throws IOException
     */
    public RobotController(Robot robot, String pathFile) throws IOException {
        this.robot = robot;

        path = new Path(Files.newInputStream(FileSystems.getDefault().getPath(pathFile)));
        LaserPropertiesResponse laserPropertiesResponse = new LaserPropertiesResponse();
        robot.getResponse(laserPropertiesResponse);
        vfhPlus = new VFHPlus(laserPropertiesResponse.getStartAngle(), laserPropertiesResponse.getEndAngle(),
                laserPropertiesResponse.getAngleIncrement());
    }

    /**
     * The method containing the loop which sends requests and responses for changing speed of the of the vehicle
     * after calling the algorithms and determining new directions and speeds. When the robot has reached the goal,
     * the loop is done and the time it took to reach the goal is given to the logger.
     *
     * @throws Exception
     */
    public void start() throws Exception {

        double curvature;
        double speedAndAngularSpeed[];

        long start = System.currentTimeMillis();
        try {
            while (running) {
                if (indexOfLastTargetNode == path.size() - 1) {

                    LocalizationResponse localizationResponse = new LocalizationResponse();
                    robot.getResponse(localizationResponse);
                    double distance = new Position(localizationResponse.getPosition()).getDistanceTo(path.get(path.size() - 1).getPosition());
                    if (distance < 0.2) {
                        logger.error("Goal reached - margin of error: " + distance + " time : " + (System.currentTimeMillis() - start) / 1000 + " s");
                        robot.drive(0, 0);
                        running = false;
                        break;
                    }
                }
                LocalizationResponse localizationResponse = new LocalizationResponse();
                robot.getResponse(localizationResponse);
                curvature = pursue();
                speedAndAngularSpeed = setSpeedAndAngularSpeed(curvature);

                robot.drive(speedAndAngularSpeed[0], speedAndAngularSpeed[1]);
            }
        } catch (Exception e) {
            robot.drive(0,0);
            running = false;
            e.printStackTrace();
        }
    }

    /**
     * Calculates the new speed and angular speed from the given curvature.
     *
     * @param curvature the curvature.
     * @return speed = [0] and angular speed = [1].
     */
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
        } else if (Math.abs(curvature) > 1) {
            speed = Math.abs(1 - Math.abs(curvature) / 2);
            angularSpeed = curvature;
        } else {
            speed = 1;
            angularSpeed = curvature;
        }
        return new double[] {speed, angularSpeed};
    }

    /**
     * The method which calls the different path tracking algorithms and obstacle avoidance algorithms when these
     * are required. See report for description of algorithm calling.
     *
     * @return a curvature.
     * @throws IOException
     */
    public double pursue() throws IOException {
        // 1. Determine vehicle position.
        LocalizationResponse localizationResponse = new LocalizationResponse();
        localizationResponse = (LocalizationResponse)robot.getResponse(localizationResponse);
        Position currentPosition = new Position(localizationResponse.getPosition());
        double heading = localizationResponse.getHeadingAngle();
        // 2. Find the point on the path closes to the vehicle, 3. Find the carrot point.
        Position carrotPosition = calcCarrotPosition(currentPosition);

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
                logger.error("No valley found. Turning sharply...");
                return 1;
            }
            if(ftcAngle != vfhAngle) {

                return vfhAngle;
            }
            vfhPlus.resetPreviousTargetSector();

        }
        vfhPlus.resetPreviousTargetSector();
        return purePursuit(heading, currentPosition, carrotPosition, distanceToCarrotPoint);
    }

    /**
     * Calculates the curvature from "Pure Pursuit".
     *
     * @param heading the heading of the vehicle.
     * @param currentPosition the position of the vehicle.
     * @param carrotPosition the position of the carrot point.
     * @param distanceToCarrotPoint the distance to the carrot point.
     * @return the calculated curvature.
     */
    private double purePursuit(double heading, Position currentPosition, Position carrotPosition, double distanceToCarrotPoint) {
        double alpha = - heading + currentPosition.getBearingTo(carrotPosition);
        double deltaX = distanceToCarrotPoint * Math.cos(alpha);
        double curvature = ((-2)*deltaX / (distanceToCarrotPoint*distanceToCarrotPoint));
        return curvature;
    }

    /**
     * Returns the error angle from the "Follow the Carrot" algorithm. Uses the constant PROPORTIONAL_GAIN.
     *
     * @param heading the heading of the vehicle.
     * @param directionToCarrotPoint the direction to the carrot point.
     * @return the error angle.
     */
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

    /**
     * Calculates the position of the carrot point by going to the closest point on the path and use the
     * getCarrotPosition method to step forward LOOK_AHEAD distance, when that distance has been reached the
     * carrot point is positioned there.
     *
     * @param vehicle_pos the position of the vehicle.
     * @return the position of the carrot point.
     */
    private Position calcCarrotPosition(Position vehicle_pos) {
        double shortestDistance = Integer.MAX_VALUE;
        DistanceData distance;
        int shortestNodeNumber = -1;
        int type = -1;
        for(int i = indexOfLastClosestNode; i < path.size() && path.get(indexOfLastClosestNode).getPosition().getDistanceTo(path.get(i).getPosition()) < 2*LOOK_AHEAD; i++) {
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
            LOOK_AHEAD += path.get(shortestNodeNumber).getPosition().getDistanceTo(position);
        }
        return getCarrotPosition(path.get(indexOfStartNode), path.get(indexOfStartNode + 1), path, LOOK_AHEAD);
    }

    /**
     * Recursive method which steps forward in the path until it has used the LOOK_AHEAD distance.
     * @param current the current PathNode.
     * @param next the next PathNode.
     * @param path the path.
     * @param lookAhead the look ahead distance left to step forward.
     * @return the position of the carrot point.
     */
    public Position getCarrotPosition(PathNode current, PathNode next, Path path, double lookAhead) {
        indexOfLastTargetNode = current.getIndex();
        if(LOOK_AHEAD == 0  || path.isLastNode(current)) {
            return current.getPosition();
        } else if(LOOK_AHEAD >= current.getPosition().getDistanceTo(next.getPosition())) {
            double new_lookAhead = lookAhead - current.getPosition().getDistanceTo(next.getPosition());
            return getCarrotPosition(next, path.get(next.getIndex() + 1), path, new_lookAhead);
        } else {
            Position carrotPosition = new Position();
            double k = LOOK_AHEAD / current.getPosition().getDistanceTo(next.getPosition());
            double x = current.getPosition().getX() + (next.getPosition().getX() - current.getPosition().getX()) * k;
            double y = current.getPosition().getY() + (next.getPosition().getY() - current.getPosition().getY()) * k;

            carrotPosition.setX(x);
            carrotPosition.setY(y);

            return carrotPosition;
        }
    }

    /**
     * This method is used to determine if a point p is closes to one of two other points or the segment between the
     * points. It also calculates the distance to the closest point.
     *
     * @param p0 The first point.
     * @param p1 The second point.
     * @param p The point which distance to the other points is to be determined.
     * @return A data type containing the distance to the closest point and whether p is closest to p0, p1 or the
     * segment between p0 and p1.
     */
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

    /**
     * Calculates the dot product between two points.
     * @param a the first point.
     * @param b the second point.
     * @return the dot product.
     */
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

    /**
     * Sets a new look ahead.
     * @param lookAhead the new look ahead.
     */
    public void setLookAhead(Double lookAhead) {
        LOOK_AHEAD = lookAhead;
        logger.error("LOOK_AHEAD: " + LOOK_AHEAD);
    }

}
