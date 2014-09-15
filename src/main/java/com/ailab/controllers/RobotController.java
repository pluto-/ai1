package com.ailab.controllers;

import com.ailab.tools.Path;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;

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
        robot.drive(0.0, PurePursuit.pursue(robot));
    }
}
