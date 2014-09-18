package com.ailab.tools;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Path {

    private int currentNode;
    private ArrayList<PathNode> path;

    public Path(InputStream stream) throws IOException {

        currentNode = 0;
        path = new ArrayList<PathNode>();

        ObjectMapper mapper = new ObjectMapper();
        path = new ArrayList<PathNode>(Arrays.asList(mapper.readValue(stream, PathNode[].class)));
    }

    public PathNode getNext() {
        PathNode node = path.get(currentNode);
        currentNode++;
        return node;
    }

    public PathNode get(int i) {
        if (i < path.size())
            return path.get(i);
        else
            return path.get(path.size() -1);
    }

    public int size() {
        return path.size();
    }
}
