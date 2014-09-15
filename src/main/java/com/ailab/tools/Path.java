package com.ailab.tools;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Path {

    int currentNode;
    ArrayList<PathNode> path;

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
}
