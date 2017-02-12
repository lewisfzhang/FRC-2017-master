# How to Calibrate.

1. Make sure you have the OpenCV library and G++ compiler.
2. Open `camera-calibration.cp`.
3. Change the output .txt file to the PixyCam number that you are calibrating. The line is: `ofstream myFile("../resources/PixyCam`...
  ex: `ofstream myFile("../resources/PixyCam4.txt");`
4. Plug in the desired Pixy into your laptop.
5. Go to terminal and change the directory to the folder `camera-calibration` in `FRC-2017`.
6. Enter the command `./compile.sh`
7. Enter the command `./a.out`
8. Click on the new window, focus the camera (by twisting the Pixy lens), and press the spacebar and then 'g'.
9. Take 8 pictures of the dot grid, from different angles, pressing the spacebar to take the picture. Make sure to only take pictures when the dot grid is recognized. You can see when the dot grid is detected by the colorful lines on the dots.
10. Once all 8 pictures are taken, it will show the undistorted image. Make sure that the calibration is working right.
11. Press Esc and close.
12. The camera matrix and distortion coefficients will be written to the .txt file you put in in step 3.

Note: If using a certain number camera on the robot, go to `Constants.java`, and change `kPixyNumber` to the desired Pixy Number.
