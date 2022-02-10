package org.firstinspires.ftc.teamcode.Autonomi;

import com.SCHSRobotics.HAL9001.system.robot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.robot.MainRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.VisionStuff.Camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "HALVision", group = "Example Programs")
public class HALVisionAuto extends BaseAutonomous {
    public @MainRobot
    Baguette robot;
    //WeirdSpinnerHalWorkaround weird = new WeirdSpinnerHalWorkaround();

    @Override
    public void main() {

        waitTime(300000);
    }
}
