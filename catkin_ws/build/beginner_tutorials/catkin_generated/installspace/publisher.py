import rospy
import cv2
import math
from std_msgs.msg import String


threshold = 20
pixel_window_apothem = 10

def getColor():
    cam = cv2.VideoCapture(cv2.CAP_V4L2)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    total_b = 0
    total_g = 0
    total_r = 0

    #path = r'/home/mgoldberg/Desktop/me588/redblock.jpg'
    #image = cv2.imread(path)
    result, image = cam.read()
    
    if result:
        dimensions = image.shape

        height = dimensions[0]
        width = dimensions[1]
        height_middle = int(height / 2)
        width_middle = int(width / 2)

        for i in range(height_middle - pixel_window_apothem, height_middle + pixel_window_apothem):
            for j in range(width_middle - pixel_window_apothem, width_middle + pixel_window_apothem):
                total_b += image[i][j][0] #cumulatively adds blue pixel values of every pixel
                total_g += image[i][j][1] #cumulative adding of green pixels
                total_r += image[i][j][2] #cumulative adding of red pixels
                
        num_pixels = (2 * pixel_window_apothem) ** 2

        blue_val = math.floor(total_b / num_pixels)
        green_val = math.floor(total_g / num_pixels)
        red_val = math.floor(total_r / num_pixels)
        
        max_val = max(blue_val, green_val, red_val)

        if(max_val == blue_val):
            return("blue")
        elif((max_val == red_val or max_val == green_val) and (abs(red_val - green_val) <= threshold)):
            return("yellow")
        elif(max_val == red_val):
            return("red")
        elif(max_val == green_val):
            return("green")
        else:
            return("no cube")
    else:
        return 'no image'

def publisher():
    pub = rospy.Publisher('color_detected', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        color = getColor()
        #rospy.loginfo(color)
        pub.publish(color)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
