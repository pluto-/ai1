package com.ailab;

import com.ailab.controllers.Robot;
import com.ailab.controllers.RobotController;
import com.ailab.tools.Path;
import com.ailab.tools.PathNode;
import com.ailab.tools.Position;
import com.ailab.tools.Util;
import com.fasterxml.jackson.databind.ObjectMapper;

import javax.swing.*;
import java.awt.*;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.util.ArrayList;

/**
 * Created by Jonas on 2014-09-15.
 */
public class Main {




    public static void main(String args[]) {


        Robot robot = new Robot("http://127.0.0.1", 50000);
        RobotController controller;


        if(args.length < 1) {
            System.out.println("Must specify a path JSON file.");
            System.exit(1);
        }

        try {
            controller = new RobotController(robot, args[0]);
            controller.start();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }
}
