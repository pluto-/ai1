package com.ailab.controllers;

import com.ailab.tools.LaserEchoesResponse;
import com.ailab.tools.LaserPropertiesResponse;

/**
 * Created by Jonas on 2014-09-22.
 */
public class LaserController {

    public static final double SOFTLIMIT = 0.5;
    public static final double HARDLIMIT = 0.2;

    public static int test(Robot robot) {
        LaserPropertiesResponse laserPropertiesResponse = new LaserPropertiesResponse();
        LaserEchoesResponse laserEchoesResponse = new LaserEchoesResponse();
        try {
            robot.getResponse(laserEchoesResponse);
            robot.getResponse(laserPropertiesResponse);
        } catch (Exception e) {
            e.printStackTrace();
        }
        System.out.println("Echo[135] distance = " + laserEchoesResponse.getEchoes()[135] + " size " + laserEchoesResponse.getEchoes().length);
        //System.out.println("Increment = " + laserPropertiesResponse.getAngleIncrement());
        //System.out.println("Position = " + laserPropertiesResponse.getPosition()[0] + " " + laserPropertiesResponse.getPosition()[1] + " " + laserPropertiesResponse.getPosition()[2]);
        //System.out.println("Orientation = " + laserPropertiesResponse.getOrientation());
        //System.out.println("Start Angle = " + laserPropertiesResponse.getStartAngle());
        //System.out.println("End Angle = " + laserPropertiesResponse.getEndAngle());
        return 0;
    }

    public static boolean testFrontCollision(Robot robot) {
        LaserEchoesResponse laserEchoesResponse = new LaserEchoesResponse();
        try {
            robot.getResponse(laserEchoesResponse);
        } catch (Exception e) {
            e.printStackTrace();
        }
        for(int i = 90; i < 180; i++) {

            if(laserEchoesResponse.getEchoes()[i] < 0.3) {
                return true;

            }
        }
        //System.out.println("Increment = " + laserPropertiesResponse.getAngleIncrement());
        //System.out.println("Position = " + laserPropertiesResponse.getPosition()[0] + " " + laserPropertiesResponse.getPosition()[1] + " " + laserPropertiesResponse.getPosition()[2]);
        //System.out.println("Orientation = " + laserPropertiesResponse.getOrientation());
        //System.out.println("Start Angle = " + laserPropertiesResponse.getStartAngle());
        //System.out.println("End Angle = " + laserPropertiesResponse.getEndAngle());
        return false;
    }

    /*public double determineCurvatureFactor(Robot robot) {

        double factor;

        LaserEchoesResponse laserEchoesResponse = new LaserEchoesResponse();
        try {
            robot.getResponse(laserEchoesResponse);
        } catch (Exception e) {
            e.printStackTrace();
        }
        double closestObstacle = Double.MAX_VALUE;
        for(int i = 100; i < 170; i++) {
            if(laserEchoesResponse.getEchoes()[i] < closestObstacle) {
                closestObstacle = laserEchoesResponse.getEchoes()[i];
            }
        }

        if(closestObstacle < SOFTLIMIT) {
            factor = (SOFTLIMIT - HARDLIMIT) / (SOFTLIMIT - closestObstacle);
        }
    }*/
}
