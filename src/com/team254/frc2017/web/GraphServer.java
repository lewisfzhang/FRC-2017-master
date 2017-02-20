package com.team254.frc2017.web;

import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.TimerTask;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import com.team254.lib.util.CrashTrackingRunnable;

import org.json.simple.JSONObject;

public class GraphServer {
    private static GraphServer mInstance = new GraphServer();
    
    private ArrayList<Socket> mClients = new ArrayList<Socket>();
    private static LinkedBlockingQueue mData = new LinkedBlockingQueue(200);
    private double timeThreshold = 50.0; // milliseconds
    // used LinkedList b/c easier to modify contents/resize
    
    private ServerSocket mServerSocket;
    
    public static GraphServer getInstance() {
        return mInstance;
    }
    
    private GraphServer() {
        try {
            mServerSocket = new ServerSocket(5801);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    public void startServer() {
        TimerTask mClientGetter = new TimerTask() {
            @Override
            public void run() {
                try {
                    Socket client = mServerSocket.accept();
                    mClients.add(client);
                } catch (IOException e) {
                    e.printStackTrace();
                }
                Timer.delay(0.05);
            }
        };
        
        TimerTask mNetworkPusher = new TimerTask() {
            @Override
            public void run() {
                if (mData.size() != 0) {
                    String output = "";
                    double firstTimestamp = ((GraphData) mData.peek()).getTimestamp();
                    for (int i = 0; i < mData.size(); i++) {
                        GraphData data = (GraphData) mData.poll();
                        if (data.getTimestamp() - firstTimestamp < timeThreshold) {
                            JSONObject outBuilder = new JSONObject();
                            outBuilder.put("Key", data.getKey());
                            outBuilder.put("Value", data.getData());
                            outBuilder.put("Timestamp", data.getTimestamp());
                            output += outBuilder.toString() + "\n";
                        }
                    }

                    try {
                        for (Socket client : mClients) {
                            if (!client.isBound()) {
                                mClients.remove(client);
                                continue;
                            }
                            PrintWriter out = new PrintWriter(client.getOutputStream(), true);
                            out.println(output);
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                Timer.delay(0.05);
            }
        };
        
        java.util.Timer clientTimer = new java.util.Timer();        
        java.util.Timer serverTimer = new java.util.Timer();
        
        clientTimer.scheduleAtFixedRate(mClientGetter,0,100);
        serverTimer.scheduleAtFixedRate(mNetworkPusher,0,100);
    }
    
    public static boolean addData(GraphData in) {
        return mData.offer(in);
    }

    public static boolean addRawData(String key, int data, double timestamp) {
        return mData.offer(new GraphData(key, data, timestamp));
    }
    
    public static  boolean addRawData(String key, double data, double timestamp) {
        return mData.offer(new GraphData(key, data, timestamp));
    }
    
    public static boolean addRawData(String key, float data, double timestamp) {
        return mData.offer(new GraphData(key, data, timestamp));
    }
}