#! /usr/bin/env python3

from math import cos, inf, pi, sin, sqrt, tan
from queue import Queue
import cv2
import numpy as np
import rclpy
import rclpy.time
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_system_default
from cv_bridge import CvBridge
from rclpy.node import Node
from riptide_msgs2.action import AlignTorpedos
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import euler_from_quaternion
from statistics import pstdev

import utils.TorpedoVision as TorpedoVision

SHOW_DEBUG_IMAGE = False

IMAGE_TOPIC = "/tempest/torpedos"
ODOM_TOPIC = "/tempest/odometry/filtered"

# CAMERA_FRAME = "tempest_left_camera_frame"
CAMERA_FRAME = "/tempest/stereo/left_link"
TORPEDO_FRAME = "/tempest/stereo/left_link" #TODO: make this the correct name

ROBOT_NAME = "tempest"
CAMERA_FOV_X = 81.25 #degrees
CAMERA_FOV_Y = 45.7 #degrees

GROUP_SIZE = 0.0625
NON_CIRCLE_PRIORITY = 1.3 #measure of how much more "important" non-circle shapes are. this will affect how the alignment prioritizes the special shapes for more points

class AlignTorpedosService(Node):
    
    def __init__(self):
        super().__init__('AlignTorpedos')
        self.actionServer = ActionServer(self, AlignTorpedos, 'AlignTorpedos', self.alignTorpedosCallback)
        self.cvBridge = CvBridge()
        
        #tf stuff
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        
        #subscribers
        self.imageSub = self.create_subscription(Image, IMAGE_TOPIC, self.imageCallback, qos_profile_system_default)
        self.odomSub = self.create_subscription(Odometry, ODOM_TOPIC, self.odomCallback, qos_profile_system_default)
        
        self.imgQueue = Queue(1)
        self.odomQueue = Queue(1)
        
        
    #
    # Utils
    #
    
    #TODO: make a util file to store all of these util methods.
    def dist(self, pt1, pt2):
        if not (len(pt1) == len(pt2)):
            self.get_logger().error("Cannot calculate distance between points of differing dimensions!")
            return -1

        sum = 0
        for i in range(0, len(pt1)):
            diff = pt2[i] - pt1[i]
            sum += diff**2 #diff squared
        
        return sqrt(sum)
    

    def distVec3(self, v1: Vector3, v2: Vector3):
        dx = v2.x - v1.x
        dy = v2.y - v1.y
        dz = v2.z - v1.z
        
        return sqrt(dx**2 + dy**2 + dz**2)
    
    
    def closestToPoint(self, rects, target):
        dist = 99999999999
        closest = None
        for (x, y, w, h) in rects:
            d = self.dist((x, y), target)
            if d < dist:
                dist = d
                closest = (x, y, w, h)
        
        return closest
    

    def rotateAboutYaw(self, pt: Vector3, angle) -> Vector3:
        x = pt.x * cos(angle) - pt.y * sin(angle)
        y = pt.x * sin(angle) + pt.y * cos(angle)
        
        res = Vector3()
        res.x = x
        res.y = y
        res.z = pt.z
        
        return res
    
    
    def averageOfVector3s(self, vecs):               
        avg = Vector3()
        for vec in vecs:
            avg.x += vec.x
            avg.y += vec.y
            avg.z += vec.z
            
        avg.x /= len(vecs)
        avg.y /= len(vecs)
        avg.z /= len(vecs)
        
        return avg
    
    
    def centerOfRect(self, rect):
        x, y, w, h = rect
        cX = x + (w/2)
        cY = y + (h/2)
        
        return (cX, cY)
    
    
    def filterPoints(self, points):
        if len(points) == 0:
            return points
        
        avg = self.averageOfVector3s(points)
        
        #get distance of each point from average point
        dists = [self.distVec3(avg, pt) for pt in points]
        avgDist = sum(dists) / len(dists)
        stdev = pstdev(dists) #standard deviation
        
        # filter out all points that are more than a certain number of stdevs away from avg
        goodPts = []
        for i in range(0, len(dists)):
            if abs(dists[i] - avgDist) < (stdev * 1.3):
                goodPts.append(points[i])
        
        return goodPts
        
            
    
    #TODO: maybe put a link to the picture of the math I did to get the formula for this
    def pixelsToMeters(self, px, imgLen, fov, dist):
        """Converts a measure of pixels in the direction of an axis (x, y) to meters.

        Args:
            px (int): Pixels to be converted to meters
            imgLen (int): if px is in the x-direction, use the image width. if px is y direction, use image height.
            fov (int): in degrees. if px is in the x-direction, use the horizontal fov. if y, use vertical.

        Returns:
            double: px converted to meters
        """
        if px == 0:
            return dist * sin(fov / 2)
        
        fovDeg = fov * (pi / 180.0)
        distPx = px / tan((px / imgLen) / fovDeg)
        pixelsPerMeter = distPx / dist
        return px / pixelsPerMeter
    
    #basically this method should be the exact python version of doTransform() in util.cpp but without transforming the quaternion and returning a pose
    def doTransform(self, coords: Vector3, transform: TransformStamped) -> Vector3:
        #rotate point with transform orientation
        rpy = euler_from_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        
        yaw = rpy[2]
        relative = self.rotateAboutYaw(coords, yaw)
        
        position = Vector3()
        position.x = relative.x + transform.transform.translation.x
        position.y = relative.y + transform.transform.translation.y
        position.z = relative.z + transform.transform.translation.z
        
        return position
    

    def transformBetweenFrames(self, pose: Vector3, fromFrame: str, toFrame: str, timeout=5.0) -> Vector3:
        startTime = self.get_clock().now()
        #attempt to look up transform until lookup succeeds or timeout is reached (or rclpy is shut down)
        while rclpy.ok() and (self.get_clock().now() - startTime).to_msg().sec < timeout:
            try:
                transform = self.tfBuffer.lookup_transform(
                    toFrame,
                    fromFrame,
                    rclpy.time.Time()
                )
                                
                return self.doTransform(pose, transform)
            except Exception as ex:
                self.get_logger().warn("Received exception while looking up transform: {}".format(ex))
        
        self.get_logger().error("Failed to look up transform from {} to {}!".format(fromFrame, toFrame))
        
        #compose error return value
        ret = Vector3()
        ret.x = inf
        ret.y = inf
        ret.z = inf
        return ret
    
    
    #transforms image pixel coordinates to global coordinates in world frame. returns (-1000, -1000, -1000) if an error happens.
    def imageToWorldCoordinates(self, center, distance, imgShape) -> Vector3:
        cX, cY = center
        imgCenX = imgShape[0] / 2
        imgCenY = imgShape[1] / 2
        
        #move image coordinate frame so that (0, 0) is the center of the image
        cX -= imgCenX
        cY -= imgCenY
        
        #convert x and y to meters. This (with the distance) will be the location of the object in camera frame.
        xMeters = self.pixelsToMeters(cX, imgShape[0], CAMERA_FOV_X, distance)
        yMeters = self.pixelsToMeters(cY, imgShape[1], CAMERA_FOV_Y, distance)
        
        print("xmeters: " + str(xMeters) + ",  ymeters: " + str(yMeters) + ",  distance: " + str(distance))
        
        #compose the vector3 describing hole position relative to camera
        relative = Vector3()
        relative.x = distance
        relative.y = xMeters
        relative.z = yMeters
                
        return self.transformBetweenFrames(relative, CAMERA_FRAME, "world")
        
        
    #
    # Callbacks and ROS stuff
    #
        
    
    def imageCallback(self, msg):
        img = self.cvBridge.imgmsg_to_cv2(msg)
        if self.imgQueue.full():
            self.imgQueue.get_nowait()
        
        self.imgQueue.put_nowait(img)
        
    
    def waitForImage(self, timeout=3.0) -> np.ndarray:
        return self.imgQueue.get(True, timeout)
    
    
    def odomCallback(self, msg):
        if self.odomQueue.full():
            self.odomQueue.get_nowait()
        
        self.odomQueue.put_nowait(msg)
        
    
    def waitForOdom(self, timeout=3.0) -> Odometry:
        return self.odomQueue.get(True, timeout)
        
        
    def alignTorpedosCallback(self, goalHandle):
        self.get_logger().info("Starting align torpedos action.")
        currentDist = goalHandle.request.currentdistance
        goalDist = goalHandle.request.goaldistance
        timeout = goalHandle.request.timeoutms
        
        if currentDist == 0:
            self.get_logger().error("Request invalid! currentdistance must be greater than 0!")
            goalHandle.abort()
            return AlignTorpedos.Result()
        
        startTime = self.get_clock().now()
        
        #list that stores lists of target coordinates. (2d list! wow!)
        coords = [] #location of circles in world frame, paired with number of segments of that shape
        
        # collect 25 good detections or time out after the specified timeout time
        while (len(coords) < 50) and (self.get_clock().now() - startTime).to_msg().sec * 1000.0 < timeout:
            img = self.waitForImage()
                        
            if img is not None:
                holes = TorpedoVision.processImage(img) #returns a list of (contour, numCorners)
                rects = [(cv2.boundingRect(hole), corners) for hole, corners in holes]
                                
                for ((x, y, w, h), segments) in rects:
                    globalCoords = self.imageToWorldCoordinates((x, y), currentDist, img.shape) #keep coordinates in global coords because world frame is more rigid than camera frame
                    coords.append((globalCoords, segments))
                    
                    print("distance: " + str(currentDist) + ",  image2world: " + str(globalCoords))
                    print()
                                        
                for ((x, y, w, h), segments) in rects:
                    cv2.circle(img, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
                                    
                if SHOW_DEBUG_IMAGE:
                    cv2.imshow("result", img)
                    cv2.waitKey(20)
                                
        if SHOW_DEBUG_IMAGE:
            cv2.destroyAllWindows()
                            
        # group all detections into groups of detections that are close to each other. The radius of each group is determined by GROUP_SIZE
        groups = [] #will be a list of (groups of coordinates that are close together, numSegments)
        for (coord, segments) in coords:
            #find all groups that the coordinate belongs to
            foundGroup = False
            for i in range(0, len(groups)):
                (group, groupSegments) = groups[i]
                if self.distVec3(coord, self.averageOfVector3s(group)) < GROUP_SIZE and segments == groupSegments:
                    groups[i][0].append(coord)
                    foundGroup = True

            if not foundGroup:
                groups.append(([coord], segments))
        
        #find the average coordinate of each group
        finalCoords = [] #will be list of (coord, numDetections, numSegments)
        for (coords, segments) in groups:
            finalCoords.append((self.averageOfVector3s(coords), len(coords), segments))
            
        print("final coords: " + str(finalCoords))
            
        #now score each group based on the shape and number of detections (circle is lower score and more detections is better)
        scores = []
        for (coord, numDetections, numSegments) in finalCoords:
            multiplier = NON_CIRCLE_PRIORITY if numSegments != 8 else 1
            scores.append(numDetections * multiplier)
            
        #find coordinate to align to by using the one with the highest score
        actionResult = AlignTorpedos.Result()
        if len(scores) > 0:
            index = scores.index(max(scores)) #index of max of scores
            targetCoord = finalCoords[index][0]
            
            self.get_logger().info("Target world coordinates: {}, {}, {}".format(targetCoord.x, targetCoord.y, targetCoord.z))
            
            #now that we have the target world coordinates, figure out where to move the robot to have the torpedos aligned
            currTorpedo = self.transformBetweenFrames(targetCoord, "world", TORPEDO_FRAME)
            print("current: " + str(currTorpedo))
            
            destTorpedo = Vector3() #goal position of of the target in torpedo frame
            destTorpedo.x = goalDist
            
            print("dest: " + str(destTorpedo))
            
            #get difference between our current coords and our goal coords
            difference = Vector3()
            difference.x = currTorpedo.x - destTorpedo.x
            difference.y = currTorpedo.y - destTorpedo.y
            difference.z = currTorpedo.z - destTorpedo.z
            
            print("diff: " + str(difference))
            
            #convert our difference into world frame by rotating the point to be aligned with world frame coords
            currentOdom = self.waitForOdom()
            orientation = currentOdom.pose.pose.orientation
            _, _, currYaw = euler_from_quaternion((orientation.w, orientation.x, orientation.y, orientation.z))
            
            diffWorld = self.rotateAboutYaw(difference, -currYaw)
            
            print("diffworld: " + str(diffWorld))
            
            if abs(diffWorld.x) == inf:
                self.get_logger().error("Coordinate transformation failed at some point! Aborting.")
                goalHandle.abort()
                return AlignTorpedos.Result()
            
            #add our world frame difference to our current position to get our ultimate world-frame destination position
            
            goalPosition = Vector3()
            goalPosition.x = currentOdom.pose.pose.position.x + diffWorld.x
            goalPosition.y = currentOdom.pose.pose.position.y + diffWorld.y
            goalPosition.z = currentOdom.pose.pose.position.z + diffWorld.z
            
            actionResult.coords = goalPosition
        
            goalHandle.succeed()
            self.get_logger().info("Torpedo alignment complete.")
        else:
            self.get_logger().error("No targets detected! Aborting.")
            goalHandle.abort()
        
        return actionResult
    
    
def main(args=None):
    rclpy.init(args=args)
    
    service = AlignTorpedosService()
    
    service.get_logger().info("Starting Align Torpedo service client.")
    rclpy.spin(service, executor=MultiThreadedExecutor())
    rclpy.shutdown()
    
    
#program starts here
if __name__ == '__main__':
    main()
