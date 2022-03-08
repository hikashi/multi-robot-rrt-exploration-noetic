#!/usr/bin/env python3
import rospy
from copy import copy
from geometry_msgs.msg import Point, PolygonStamped, Polygon, PointStamped
from std_msgs.msg import Header
from time import time

#############################################################
recordedPoints = []

def recordedPointsCallback(data):
    global recordedPoints, dataLength
    
    if len(recordedPoints) < 4:
        tempPoints = Point()
        tempPoints.x = data.point.x
        tempPoints.y = data.point.y
        tempPoints.z = data.point.z
        recordedPoints.append(tempPoints)
        dataLength += 1
    # else:
    #     print('data point more than designated data')
#############################################################
def node():
    # decide the global parameter for the data and workign on the necessary things for the data
    global recordedPoints, dataLength
    rospy.init_node('exploration_boundary', anonymous=False)

    # define the rosparam
    mapFrame      = rospy.get_param('~map_frame', 'map')   # the name frame boundary going to be published on
    n_point       = rospy.get_param('~n_point', 4) # the number of the boundary point going to be created
    topicInput    = rospy.get_param('~topicInput', '/clicked_point') # the number of the boundary point going to be created
    topicOutput   = rospy.get_param('~topicOutput', '/exploration_boundary') # specify the topic which the node is going to published into the list
    frequency     = rospy.get_param('~frequency', 1.0)     # number of the rate listening to the ros param
    timeInterval  = rospy.get_param('~timeInterval', 5.0)   # the name frame boundary going to be published on

    # define the odom capture time


    # preprocess some of the data
    rate = rospy.Rate(frequency)
    dataLength = n_point
    rospy.Subscriber(topicInput, PointStamped, recordedPointsCallback)


    # start the main function
    while len(recordedPoints) < n_point:
        # rospy.loginfo('Please input %d points to begin RRT exploration boundary' %(n_point)) 
        rospy.sleep(1.0)
        print('Entered %d of %d to start exploration' %(len(recordedPoints), n_point)) 
        pass

    rospy.loginfo('boundary polygon: received all ' + str(n_point) + ' points from the user')
    # need to determine which one is the continuous
    #! to be implemented in the next iteration *****

    # check only 4 points input by the user.
    boundaryPolygon  = rospy.Publisher(topicOutput, PolygonStamped, queue_size=10)
    start_time = rospy.get_rostime().secs
    nextTimeReport = start_time + timeInterval
    rospy.loginfo('--- >>> the exploration starts at time: %.2f ' %(start_time))
    while not rospy.is_shutdown():
        # now start workign on publishing theboudnary geometry point
        header = Header()
        boundaryList = PolygonStamped()
        boundaryPoints = Polygon()
        
        boundaryPoints.points = copy(recordedPoints)
        boundaryList.polygon = boundaryPoints

        header.frame_id = mapFrame
        header.stamp = rospy.Time.now()
        boundaryList.header = header

        boundaryPolygon.publish(boundaryList)
        
        if nextTimeReport <= rospy.get_rostime().secs:
            rospy.loginfo('--- >>> current time: %.2f    time elapsed: %.2f' %(rospy.get_rostime().secs, rospy.get_rostime().secs-start_time))
            nextTimeReport = rospy.get_rostime().secs + timeInterval


#############################################################
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 