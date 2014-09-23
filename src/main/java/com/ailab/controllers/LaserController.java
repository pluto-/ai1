package com.ailab.controllers;

import com.ailab.tools.LaserEchoesResponse;
import com.ailab.tools.LaserPropertiesResponse;

/**
 * Created by Jonas on 2014-09-22.
 */
public class LaserController {

    public static final double SOFTLIMIT = 0.5;
    public static final double HARDLIMIT = 0.2;

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
        return false;
    }
}
