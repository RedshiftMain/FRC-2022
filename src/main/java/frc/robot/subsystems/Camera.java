/*package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.cscore.VideoSource;
//import edu.wpi.first.wpilibj.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.VideoMode.PixelFormat;

public class Camera{
    static UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    public static CameraServer cameraServer;
    static boolean bool = false;

    public static void videoToDashboard() {

        MjpegServer mjpegServer1 = new MjpegServer("USB Camera 0", 1181);

        mjpegServer1.setSource(usbCamera);
        CvSink cvSink = new CvSink("opencv_USB Camera 0");
        cvSink.setSource(usbCamera);
      
        CvSource outputStream = new CvSource("usbCamera", PixelFormat.kMJPEG, 640, 480, 15);
        MjpegServer mjpegServer2 = new MjpegServer("cvSink", 1182);
        mjpegServer2.setSource(outputStream);
        SmartDashboard.putString("Camera feed", "http://10.36.76.2:"+mjpegServer2.getPort());
        //Following used to remove warnings. Functionally does nothing
        if(bool){
            mjpegServer1.close();
            cvSink.close();
            mjpegServer2.close();
        }
        
    }

    
    
}

*/