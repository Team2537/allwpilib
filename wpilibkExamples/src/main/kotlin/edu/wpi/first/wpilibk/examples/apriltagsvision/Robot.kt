// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.wpilibk.examples.apriltagsvision

import edu.wpi.first.apriltag.AprilTagDetector
import edu.wpi.first.apriltag.AprilTagPoseEstimator
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.TimedRobot
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

/**
 * This is a demo program showing the detection of AprilTags. The image is acquired from the USB
 * camera, then any detected AprilTags are marked up on the image and sent to the dashboard.
 *
 *
 * Be aware that the performance on this is much worse than a coprocessor solution!
 */
object Robot : TimedRobot() {
    /** Called once at the beginning of the robot program.  */
    init {
        val visionThread = Thread { this.apriltagVisionThreadProc() }
        visionThread.isDaemon = true
        visionThread.start()
    }

    fun apriltagVisionThreadProc() {
        val detector = AprilTagDetector()
        
        // look for tag36h11, correct 1 error bit (hamming distance 1)
        // hamming 1 allocates 781KB, 2 allocates 27.4 MB, 3 allocates 932 MB
        // max of 1 recommended for RoboRIO 1, while hamming 2 is feasible on the RoboRIO 2
        detector.addFamily("tag36h11", 1)

        // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
        // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
        val poseEstConfig =
            AprilTagPoseEstimator.Config(
                0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522
            )
        val estimator = AprilTagPoseEstimator(poseEstConfig)

        // Get the UsbCamera from CameraServer
        val camera = CameraServer.startAutomaticCapture()
        // Set the resolution
        camera.setResolution(640, 480)

        // Get a CvSink. This will capture Mats from the camera
        val cvSink = CameraServer.getVideo()
        // Setup a CvSource. This will send images back to the Dashboard
        val outputStream = CameraServer.putVideo("Detected", 640, 480)

        // Mats are very memory expensive. Lets reuse these.
        val mat = Mat()
        val grayMat = Mat()

        // Instantiate once
        val tags = ArrayList<Long>()
        val outlineColor = Scalar(0.0, 255.0, 0.0)
        val crossColor = Scalar(0.0, 0.0, 255.0)

        // We'll output to NT
        val tagsTable = NetworkTableInstance.getDefault().getTable("apriltags")
        val pubTags = tagsTable.getIntegerArrayTopic("tags").publish()

        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0L) {
                // Send the output the error.
                outputStream.notifyError(cvSink.error)
                // skip the rest of the current iteration
                continue
            }

            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY)

            val detections = detector.detect(grayMat)

            // have not seen any tags yet
            tags.clear()

            for (detection in detections) {
                // remember we saw this tag
                tags.add(detection.id.toLong())

                // draw lines around the tag
                for (i in 0..3) {
                    val j = (i + 1) % 4
                    val pt1 = Point(detection.getCornerX(i), detection.getCornerY(i))
                    val pt2 = Point(detection.getCornerX(j), detection.getCornerY(j))
                    Imgproc.line(mat, pt1, pt2, outlineColor, 2)
                }

                // mark the center of the tag
                val cx = detection.centerX
                val cy = detection.centerY
                val ll = 10
                Imgproc.line(mat, Point(cx - ll, cy), Point(cx + ll, cy), crossColor, 2)
                Imgproc.line(mat, Point(cx, cy - ll), Point(cx, cy + ll), crossColor, 2)

                // identify the tag
                Imgproc.putText(
                    mat,
                    detection.id.toString(),
                    Point(cx + ll, cy),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    crossColor,
                    3
                )

                // determine pose
                val pose = estimator.estimate(detection)

                // put pose into dashboard
                val rot = pose.rotation
                tagsTable
                    .getEntry("pose_" + detection.id)
                    .setDoubleArray(
                        doubleArrayOf(
                            pose.x, pose.y, pose.z, rot.x, rot.y, rot.z
                        )
                    )
            }

            // put list of tags onto dashboard
            pubTags.set(tags.stream().mapToLong { obj: Long -> obj.toLong() }.toArray())

            // Give the output stream a new image to display
            outputStream.putFrame(mat)
        }

        pubTags.close()
        detector.close()
    }
}