package com.ailab;

import com.ailab.controllers.Robot;
import com.ailab.controllers.RobotController;

import java.io.IOException;

/**
 * Created by Jonas on 2014-09-15.
 */
public class Main {


    public static void main(String args[]) {
        Robot robot = new Robot("http://127.0.0.1", 50000);
        RobotController controller;

        if (args.length < 1) {
            System.out.println("Must specify a path JSON file.");
            System.exit(1);
        }

        try {
            controller = new RobotController(robot, args[0]);
            if (args.length > 1) {
                controller.setLookahead(Double.parseDouble(args[1]));
            }
            controller.start();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }


    }
}
