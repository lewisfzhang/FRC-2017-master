package com.team254.frc2017.web;

public class GraphData {
    String key;
    double data;
    double timestamp;
    public GraphData (String key, double data, double timestamp) {
        this.key = key;
        this.data = data;
        this.timestamp = timestamp;
    }
    
    public String getKey() {
        return key;
    }
    
    public double getData() {
        return data;
    }
    
    public double getTimestamp() {
        return timestamp;
    }
}  