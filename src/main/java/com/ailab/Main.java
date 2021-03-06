package com.ailab;

import com.ailab.controllers.Robot;
import com.ailab.controllers.RobotController;

/**
 * This class contains the main method which starts the program. It sets 50000 as the port number on local host and
 * takes a path to the json file as parameter. The user can also specify a look ahead distance as parameter.
 *
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
            if (args.length > 1) {
                controller = new RobotController(robot, args[0],(Double.parseDouble(args[1])));
            } else {
                controller = new RobotController(robot, args[0]);
            }
            controller.start();
        } catch (Exception e) {
            e.printStackTrace();
        }


    }
}
