package com.team254.lib.util;

import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.subsystems.Drive;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class WebcamServer implements Loop {    
    private int mCamXRes = 320;
    private int mCamYRes = 240;
    
    private int mThreshold = 127;
    
    private CameraServer cvServer;
    private CvSink cvSink;
    private CvSource cvSource;
    private Mat source = new Mat();
    private Mat output = new Mat();
    
    public void onStart(double timestamp) {
        cvServer = CameraServer.getInstance();
        UsbCamera camera = cvServer.startAutomaticCapture();
        cvSink = cvServer.getVideo();
        cvSource = cvServer.putVideo("Output", mCamXRes, mCamYRes);
        camera.setResolution(mCamXRes, mCamYRes);
        camera.setExposureManual(0);
    }
    
    public void onLoop(double timestamp) {
        cvSink.grabFrame(source);
        Mat tempIn = new Mat();
        Mat tempOut = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.cvtColor(source, tempIn, Imgproc.COLOR_BGR2GRAY);
        
        Imgproc.threshold(tempIn, tempOut, mThreshold, 255, Imgproc.THRESH_BINARY);
        tempIn = tempOut;
        Imgproc.findContours(tempIn, contours, tempOut, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        
        ArrayList<Rect> rects = new ArrayList<Rect>();
        
        System.out.println("Num Targets Detected: " + contours.size());
        for (int i = 0; i < contours.size(); i++) {
            if (!(contours.get(i) == null || contours.isEmpty())) {
                MatOfPoint contour = contours.get(i);
                MatOfInt hull = new MatOfInt();
                Imgproc.convexHull(contour, hull);
                rects.add(Imgproc.boundingRect(contour));
                Imgproc.rectangle(output, new Point(rects.get(i).x, rects.get(i).y), 
                        new Point(rects.get(i).x + rects.get(i).width, rects.get(i).y + rects.get(i).height), 
                        new Scalar(0,255,0));
                System.out.println("Block: " + i + "\nWidth: " + rects.get(i).width + "\nHeight: " + 
                        rects.get(i).height + "\nX Pos: " + rects.get(i).x + "\nY Pos: " + rects.get(i).y + "\n");
            }
        }
        
//        cvSource.putFrame(output);
        System.out.println("Size: " + rects.size());
        
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Override
    public void onStop(double timestamp) {
    }
    
}
