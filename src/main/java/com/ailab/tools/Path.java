package com.ailab.tools;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Path {

    private ArrayList<PathNode> path;
    static final Logger logger = LogManager.getLogger(Path.class.getName());

    public Path(InputStream stream) throws IOException {

        path = new ArrayList<PathNode>();

        ObjectMapper mapper = new ObjectMapper();
        path = new ArrayList<PathNode>(Arrays.asList(mapper.readValue(stream, PathNode[].class)));
    }

    public PathNode get(int i) {
        if (i < path.size()) {
            return path.get(i);
        } else {
            return path.get(path.size() - 1);
        }
    }

    public boolean isLastNode(PathNode pathNode) {
        return pathNode.getIndex() == path.size() - 1;
    }

    public int size() {
        return path.size();
    }
}
