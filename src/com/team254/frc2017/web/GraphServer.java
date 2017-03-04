package com.team254.frc2017.web;

import java.net.URI;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.drafts.Draft;
import org.java_websocket.handshake.ServerHandshake;

public class GraphServer extends WebSocketClient {

	public GraphServer(URI serverUri, Draft draft) {
		super(serverUri, draft);
	}

	public GraphServer(URI serverURI) {
		super(serverURI);
	}

	@Override
	public void onOpen(ServerHandshake handshakedata) {
		System.out.println("new connection opened");
	}

	@Override
	public void onClose(int code, String reason, boolean remote) {
		System.out.println("closed with exit code " + code + " additional info: " + reason);
		isConnected = false;
	}

	@Override
	public void onMessage(String message) {
		System.out.println("Received message: " + message);
		isConnected = true;
	}

	@Override
	public void onError(Exception ex) {
		System.err.println("an error occurred:" + ex);
	}

	private boolean isConnected;
	
	public static boolean isConnected(GraphServer mGraphServer) {
	    return mGraphServer.isConnected;
	}
	
	public void send(GraphServer mGraphServer, GraphData mGraphData) {
	    mGraphServer.send("\"" + mGraphData.getChart() + "\"");
	    mGraphServer.send("\"" + mGraphData.getKey() + "\"");
	    mGraphServer.send(Double.toString(mGraphData.getData()));
	    mGraphServer.send(Double.toString(mGraphData.getTimestamp()));
	}
}
