package com.ailab.tools;

import com.fasterxml.jackson.annotation.JsonProperty;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;

/**
 * Created by Patrik on 2014-09-15.
 */
public class PathNode {


    @JsonProperty("Status")
    private int status;
    @JsonProperty("Timestamp")
    private int timestamp;
    @JsonProperty("Pose")
    private Pose pose;

    private int index;
    private static int globalIndex = 0;
    static final Logger logger = LogManager.getLogger(PathNode.class.getName());

    public PathNode() {
        index = globalIndex++;
    }

    public int getIndex() {
        return index;
    }

    public Pose getPose() {
        return pose;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public int getStatus() {
        return status;
    }

    public void setStatus(int status) {
        this.status = status;
    }

    public int getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(int timestamp) {
        this.timestamp = timestamp;
    }

    public Position getPosition() {
        return pose.getPosition();
    }

    public String toString() {
        return ("X: " + pose.getPosition().getX() + " Y: " + pose.getPosition().getY());
    }
}
