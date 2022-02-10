package org.firstinspires.ftc.teamcode.VisionStuff;

import com.SCHSRobotics.HAL9001.system.robot.Camera;

//import org.opencv.android.OpenCVLoader;

import org.firstinspires.ftc.teamcode.Baguette;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import com.SCHSRobotics.HAL9001.system.robot.HALPipeline;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.system.robot.VisionSubSystem;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class Vision extends SubSystem {

    public static final int LEFT_BARRIER = 120;
    public static final int RIGHT_BARRIER = 200;

    public static final int UPPER_BARRIER = 100;
    //public static final int LOWER_BARRIER = 100;

    public OpenCvCamera camera;
    public Vision(Robot robot) {
        super(robot);



    }

    @Override
    public void init() {
        //OpenCVLoader.initDebug();
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);
        camera.setPipeline(new TestPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {

                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }


    public class TestPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {

            Mat binary = new Mat();
            Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HLS);
            Core.inRange(input, new Scalar(0, 0, 0), new Scalar(255, 40, 255), input);


            input = input.submat(new Rect(0, UPPER_BARRIER, input.rows()-1, input.height()-(UPPER_BARRIER+1)));
            Imgproc.morphologyEx(input, input, Imgproc.MORPH_CLOSE, Mat.ones(7,7, CvType.CV_32F));

            Mat hierarchy = new Mat();

            List<MatOfPoint> contours = new ArrayList<>();
            List<Rect> rects = new ArrayList<>();
            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            System.out.println(contours.size());

            Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2BGR);
            //iterates through every contour, will detect if object is a  duck or cube via edge detection. will draw contours if enabled.

            for (int i = 0; i < contours.size(); i++) {
                Rect currentRect = Imgproc.boundingRect(contours.get(i));
                rects.add(currentRect);
                //if (currentRect.area() < areaReqToDetect) continue;
                Point rectCenter = new Point(rects.get(i).x + rects.get(i).width / 2, rects.get(i).y + rects.get(i).height / 2);

                //Imgproc.drawContours(input, contours, i, new Scalar(0, 255, 0), 5);
            }
            Imgproc.line(input, new Point(LEFT_BARRIER, 0), new Point(LEFT_BARRIER, input.rows()-1), new Scalar(255, 0, 0), 2);

            Imgproc.rectangle(input, new Rect(0, 0, input.width()-1, input.height()-1), new Scalar(0,0,0), -1);
            int i = 0;
            int max = 0;
            for (int j = 0; j < rects.size(); j++) {
                Rect rect = rects.get(j);
                Imgproc.rectangle(input, rects.get(j), new Scalar(255, 0, 0), -1);
                if (max <= (rect.area())) {
                    max = Math.max(max, (int) rect.area());
                    i = j;
                }
            }

            Baguette.realPosition = Baguette.TSEPosition.RIGHT;
            if (rects.size() >= 1) Baguette.biggest = rects.get(i);


            if (rects.size() >= 1) {
                if(rects.get(i).area() > 4000 ) Imgproc.rectangle(input, rects.get(i), new Scalar(0,255,0), -1);
                else {
                    Imgproc.rectangle(input, new Point(0,0), new Point(input.cols() - 1, input.rows() -1), new Scalar(255, 255, 0));
                    return input;
                }
            }
            else {
                Imgproc.rectangle(input, new Point(0,0), new Point(input.cols() - 1, input.rows() -1), new Scalar(255, 255, 0));
                return input;
            }


            if (rects.get(i).x < LEFT_BARRIER) {
                Baguette.realPosition = Baguette.TSEPosition.LEFT;
                Imgproc.rectangle(input, new Point(0,0), new Point(LEFT_BARRIER, input.rows() -1), new Scalar(255, 255, 0));
                return input;
            }

            Imgproc.rectangle(input, new Point(LEFT_BARRIER, 0), new Point(input.cols()-1, input.rows() -1), new Scalar(255, 255, 0));
            Baguette.realPosition = Baguette.TSEPosition.CENTER;
            return input;
        }
    }
}
