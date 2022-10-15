# Road Monitor Version 1.0

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
#import math
import datetime
import cv2
from influxdb import InfluxDBClient

# place a prompt on the displayed image
def prompt_on_image(txt):
    global image
    cv2.putText(image, txt, (10, 35),
    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
     
# calculate speed from pixels and time
def get_speed(pixels, mperpixel, secs):
    if secs > 0.0:
        return ((pixels * mperpixel)/ secs) * 3.6
    else:
        return 0.0
 
# calculate elapsed seconds
def secs_diff(endTime, begTime):
    diff = (endTime - begTime).total_seconds()
    return diff

# record speed in .csv format
def record_speed(res):
    global csvfileout
    f = open(csvfileout, 'a')
    f.write(res+"\n")
    f.close

# mouse callback function for drawing capture area
def draw_rectangle(event,x,y,flags,param):
    global ix,iy,fx,fy,drawing,setup_complete,image, org_image, prompt
 
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y
 
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            image = org_image.copy()
            prompt_on_image(prompt)
            cv2.rectangle(image,(ix,iy),(x,y),(0,255,0),2)
  
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        fx,fy = x,y
        image = org_image.copy()
        prompt_on_image(prompt)
        cv2.rectangle(image,(ix,iy),(fx,fy),(0,255,0),2)
        
# Set some constants dependent 
MPERPIXEL_L_TO_R = 0.024
MPERPIXEL_R_TO_L = 0.029
MIN_SPEED = 10
MAX_SPEED = 100
PICTURE_LIMIT = 65
SAVE_CSV = True
SAVE_PICTURE = True
#THRESHOLD = 15
MIN_AREA = 500
#BLURSIZE = (15,15)
IMAGEWIDTH = 640
IMAGEHEIGHT = 480
RESOLUTION = [IMAGEWIDTH,IMAGEHEIGHT]
FPS = 30
SHOW_BOUNDS = True
SHOW_IMAGE = True
MEASURMENTS_REQUIRED = 4

# the following enumerated values are used to make the program more readable
WAITING = 0
TRACKING = 1
SAVING = 2
UNKNOWN = 0
LEFT_TO_RIGHT = 1
RIGHT_TO_LEFT = 2
TOO_FEW_MEASUREMENTS = 1
DIRECTION_CHANGED = 2
DATA_OK = 0

# state maintains the state of the speed computation process
# if starts as WAITING
# the first motion detected sets it to TRACKING
 
# if it is tracking and no motion is found or the x value moves
# out of bounds, state is set to SAVING and the speed of the object
# is calculated
# initial_x holds the x value when motion was first detected
# last_x holds the last x value before tracking was was halted
# depending upon the direction of travel, the front of the
# vehicle is either at x, or at x+w 
# (tracking_end_time - tracking_start_time) is the elapsed time
# from these the speed is calculated and displayed 
 
state = WAITING
direction = UNKNOWN
last_dir = UNKNOWN
initial_x = 0
last_x = 0
 
#-- other values used in program
measurements = 0
abs_chg = 0
kmh = 0
secs = 0.0
ix,iy = -1,-1
fx,fy = -1,-1
drawing = False
setup_complete = False
tracking = False
text_on_image = 'No cars'
prompt = ''
total_no_of_cars = 0
hour_sum_of_speed = 0
day_sum_of_speed = 0
total_sum_of_speed = 0
day_avg_speed = 0
hour_avg_speed = 0
total_avg_speed = 0
cap_time = datetime.datetime.now()
logging_hour = cap_time.strftime('%H')
logging_day = cap_time.strftime('%d')
save_hour_stats = False
save_day_stats = False
cars_per_hour = 0
cars_per_day = 0
cars_in_total = 0
tracking_lost = 0

# initialize the camera. Adjust vflip and hflip to reflect your camera's orientation
camera = PiCamera()
camera.resolution = RESOLUTION
camera.framerate = FPS
camera.vflip = True
camera.hflip = True

rawCapture = PiRGBArray(camera, size=camera.resolution)
# allow the camera to warm up
time.sleep(0.9)

# create an image window and place it in the upper left corner of the screen
cv2.namedWindow("Road Monitor")
cv2.moveWindow("Road Monitor", 10, 40)

# call the draw_rectangle routines when the mouse is used
cv2.setMouseCallback('Road Monitor',draw_rectangle)
 
# grab a reference image to use for drawing the monitored area's boundry
camera.capture(rawCapture, format="bgr", use_video_port=True)
image = rawCapture.array
rawCapture.truncate(0)
org_image = image.copy()

if SAVE_CSV:
    csvfileout = "carspeed_{}.cvs".format(datetime.datetime.now().strftime("%Y%m%d_%H%M"))
    record_speed('Date,Day,Time,Speed,Image')
else:
    csvfileout = ''

prompt = "Define the monitored area - press 'c' to continue" 
prompt_on_image(prompt)
 
# wait while the user draws the monitored area's boundry
while not setup_complete:
    cv2.imshow("Road Monitor",image)
 
    #wait for for c to be pressed  
    key = cv2.waitKey(1) & 0xFF
  
    # if the `c` key is pressed, break from the loop
    if key == ord("c"):
        break

# the monitored area is defined, time to move on
prompt = "Press 'q' to quit" 
 
# since the monitored area's bounding box could be drawn starting 
# from any corner, normalize the coordinates
 
if fx > ix:
    upper_left_x = ix
    lower_right_x = fx
else:
    upper_left_x = fx
    lower_right_x = ix
 
if fy > iy:
    upper_left_y = iy
    lower_right_y = fy
else:
    upper_left_y = fy
    lower_right_y = iy
     
monitored_width = lower_right_x - upper_left_x
monitored_height = lower_right_y - upper_left_y
 
print("Monitored area:")
print(" upper_left_x {}".format(upper_left_x))
print(" upper_left_y {}".format(upper_left_y))
print(" lower_right_x {}".format(lower_right_x))
print(" lower_right_y {}".format(lower_right_y))
print(" monitored_width {}".format(monitored_width))
print(" monitored_height {}".format(monitored_height))
print(" monitored_area {}".format(monitored_width * monitored_height))
 
# Capture frames from the camera using capture_continuous
# This keeps the picamera in capture mode - it doesn't need
# to prep for each frame's capture.

object_detector = cv2.createBackgroundSubtractorMOG2()

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #initialize the timestamp
    timestamp = datetime.datetime.now()
    
    # grab the raw NumPy array representing the image 
    image = frame.array
 
    # crop area defined by [y1:y2,x1:x2]
    roi = image[upper_left_y:lower_right_y,upper_left_x:lower_right_x]
 
    mask = object_detector.apply(roi)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
    
    # look for motion 
    motion_found = False
    biggest_area = 0
 
    # examine all the contours in the frame and find the largest one
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if (area > MIN_AREA) and (area > biggest_area):
            cv2.drawContours(roi, [cnt], -1, (255, 255, 0), 1)
            (x1, y1, w1, h1) = cv2.boundingRect(cnt)
            cv2.rectangle(roi, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)
            biggest_area = area
            motion_found = True
            x = x1
            y = y1
            h = h1
            w = w1
            #print("{0:4d}  {1:4d}  {2:4d}  {3:4d}  {4:4}".format(x,y,w,h,biggest_area))
    
    if motion_found:
        # intialize motion tracking
        if state == WAITING:
            state = TRACKING
            if x == 0:
                initial_x = x + w
            else:
                initial_x = x
            initial_time = timestamp
            last_x = x
            last_kmh = 0
            direction = UNKNOWN
            last_dir = UNKNOWN
            measurements = 0
            data_code = DATA_OK
            text_on_image = 'Tracking'
            print(text_on_image)
            print("x-chg    Secs      kmh  x-pos width y-pos dir meas")
        # tracking ongoing
        else:
            # compute the lapsed time
            secs = secs_diff(timestamp,initial_time)

            if secs >= 15:
                state = WAITING
                direction = UNKNOWN
                text_on_image = 'No Car Detected'
                motion_found = False
                biggest_area = 0
                rawCapture.truncate(0)
                print('Resetting')
                continue

            if state == TRACKING:       
                if x >= last_x:
                    direction = LEFT_TO_RIGHT
                    abs_chg = x + w - initial_x
                    mperpixel = MPERPIXEL_L_TO_R
                else:
                    direction = RIGHT_TO_LEFT
                    abs_chg = initial_x - x
                    mperpixel = MPERPIXEL_R_TO_L

                # If this is the first measurement, set the direction
                if last_dir == UNKNOWN:
                    last_dir = direction             
                else:
                    if last_dir != direction:
                        # Direction has changed, disqualify data
                        #print(last_dir)
                        data_code = DIRECTION_CHANGED
                        
                kmh = get_speed(abs_chg,mperpixel,secs)
                measurements += 1
                print("{0:4d}  {1:7.2f}  {2:7.0f}   {3:4d}  {4:4d} {5:5d} {6:3d}  {7:3d}".format(abs_chg,secs,kmh,x,w,y,direction,measurements))
                
                # Is front of object outside the monitired boundary then stop motion tracking
                if ((x <= 2) and (direction == RIGHT_TO_LEFT)) \
                        or ((x+w >= monitored_width - 2) and (direction == LEFT_TO_RIGHT)):

                    if ((kmh > MIN_SPEED) and (kmh < MAX_SPEED)):    # This is a valid measurement
                        
                        if SAVE_PICTURE and (kmh > PICTURE_LIMIT):
                            # timestamp the image
                            cv2.putText(image, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
                                (10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 1)
                            # write the speed: first get the size of the text
                            size, base = cv2.getTextSize( "%.0f kmh" % kmh, cv2.FONT_HERSHEY_SIMPLEX, 2, 3)
                            # then center it horizontally on the image
                            cntr_x = int((IMAGEWIDTH - size[0]) / 2) 
                            cv2.putText(image, "%.0f kmh" % kmh,
                                (cntr_x , int(IMAGEHEIGHT * 0.2)), cv2.FONT_HERSHEY_SIMPLEX, 2.00, (0, 255, 0), 3)
                            # and save the image to disk
                            # imageFilename = "car_at_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"
                            # use the following image file name if you want to be able to sort the images by speed
                            imageFilename = "car_at_%02.0f" % kmh + "_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"
                            cv2.imwrite(imageFilename,image)
                        
                        if SAVE_CSV:
                            cap_time = datetime.datetime.now()
                            imageFilename = "car_at_%02.0f" % kmh + "_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"
                            record_speed(cap_time.strftime("%Y.%m.%d")+','+cap_time.strftime('%A')+','+\
                               cap_time.strftime('%H%M')+','+("%.0f" % kmh) + ','+imageFilename)
                            #Save to file                         
                            
                        # Evaluate number of measurements
                        if measurements < MEASURMENTS_REQUIRED:
                            data_code = TOO_FEW_MEASUREMENTS

                        # Measurement is valid and counters to be incremented           
                        if data_code == DATA_OK:
                                                       
                            # Check if time to save hour stats
                            if logging_hour != cap_time.strftime('%H'): 
                                save_hour_stats = True
                                hour_stats = [
                                    {
                                        "measurement" : "hour_stats",
                                        "fields" : {
                                            "cars_per_hour": cars_per_hour,
                                            "hour_avg_speed": float(hour_avg_speed)
                                        }
                                    }
                                ]
                                logging_hour = cap_time.strftime('%H')
                                cars_per_hour = 1 # reset hour counters
                                hour_avg_speed = kmh
                                hour_sum_of_speed = kmh
                                
                                new_logging_hour = float(logging_hour)
                                
                                # Check if time to save day stats
                                if new_logging_hour == 0:
                                    save_day_stats = True
                                    day_stats = [
                                        {
                                            "measurement" : "day_stats_2",
                                            "fields" : {
                                                "cars_per_day": cars_per_day,
                                                "day_avg_speed": float(day_avg_speed)
                                            }
                                        }
                                    ]
                                    cars_per_day = 1
                                    day_sum_of_speed = kmh
                                    day_avg_speed = kmh
                                else:
                                    cars_per_day += 1
                                    day_sum_of_speed = day_sum_of_speed + kmh
                                    day_avg_speed = day_sum_of_speed / cars_per_day
                            
                            else:
                                cars_per_hour += 1
                                hour_sum_of_speed = hour_sum_of_speed + kmh
                                hour_avg_speed = hour_sum_of_speed / cars_per_hour
                                
                                cars_per_day += 1
                                day_sum_of_speed = day_sum_of_speed + kmh
                                day_avg_speed = day_sum_of_speed / cars_per_day
                                
                            cars_in_total += 1
                            #total_sum_of_speed = total_sum_of_speed + kmh
                            #total_avg_speed = total_sum_of_speed / cars_in_total
                            
                        # Print some debug/stats
                        print("Cars per hour:           {}".format(cars_per_hour))
                        print("Cars per day:            {}".format(cars_per_day))
                        print("Cars in total:           {}".format(cars_in_total))
                        print("Last speed recorded:     {}".format("%.2f" % kmh))                        
                        print("Average speed per hour:  {}".format("%.2f" % hour_avg_speed))
                        print("Average speed per day:   {}".format("%.2f" % day_avg_speed))
                        #print("Average speed in total:  {}".format("%.2f" % total_avg_speed))
                        print("Last direction recorded: {}".format(direction))
                        print("Measurements:            {}".format(measurements))
                        print("Data code:               {}".format(data_code))
                        #print("Tracking lost:           {}".format(tracking_lost))
                        
                        speed_data = [
                            {
                                "measurement" : "car_speed",
                                "tags" : {
                                    "direction": direction,
                                    "data_code": data_code
                                },
                                "fields" : {
                                    "speed": float(kmh)
                                  #  "total_avg_speed" : float(total_avg_speed),
                                }
                            }
                        ]
                        client = InfluxDBClient('localhost', 8086, '********', '********', '********')

                        client.write_points(speed_data)
                        
                        if save_hour_stats == True:
                            client.write_points(hour_stats)
                            save_hour_stats = False
                            
                        if save_day_stats == True:
                            client.write_points(day_stats)
                            save_day_stats = False
                        
                    state = SAVING #Why is this needed? Check later
                
                # if the object hasn't reached the end of the monitored area, just remember the speed, 
                # its last position and direction
                last_kmh = kmh
                last_x = x
                last_dir = direction
    else:
        if state != WAITING:
            state = WAITING
            text_on_image = 'Waiting for cars'
            print("Tracking lost")
            tracking_lost += 1 

    # Print text and timestamp on frame
    cv2.putText(image, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
        (10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(image, "Road Status: {}".format(text_on_image), (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,0.35, (0, 255, 0), 1)
    cv2.putText(image, "Upper left:  {0:3d},{1:3d}".format(upper_left_x, upper_left_y), (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,0.35, (255, 255, 255), 1)
    cv2.putText(image, "Lower right: {0:3d},{1:3d}".format(lower_right_x, lower_right_y), (10, 75),
        cv2.FONT_HERSHEY_SIMPLEX,0.35, (255, 255, 255), 1)
    cv2.putText(image, "Width:       {0:3d} px".format(monitored_width), (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,0.35, (255, 255, 255), 1)
    cv2.putText(image, "Dist L<-R:   {}m".format("%.2f" % (monitored_width * MPERPIXEL_R_TO_L)), (10, 110),
        cv2.FONT_HERSHEY_SIMPLEX,0.35, (255, 255, 255), 1)
    cv2.putText(image, "Dist L->R:   {}m".format("%.2f" % (monitored_width * MPERPIXEL_L_TO_R)), (10, 125),
        cv2.FONT_HERSHEY_SIMPLEX,0.35, (255, 255, 255), 1)

    if SHOW_BOUNDS:
        # Print left and right boundary on frame
        cv2.line(image,(upper_left_x,upper_left_y),(upper_left_x,lower_right_y),(0, 255, 0))
        cv2.line(image,(lower_right_x,upper_left_y),(lower_right_x,lower_right_y),(0, 255, 0))
       
        # show the frame and check for a keypress
    if SHOW_IMAGE:
        prompt_on_image(prompt)
        cv2.imshow("Road Monitor", image)
        #cv2.imshow("Mask", mask)

    #if state == WAITING:
        #last_x = 0

    #state=WAITING;
    key = cv2.waitKey(1) & 0xFF
      
        # if the `q` key is pressed, break from the loop and terminate processing
    if key == ord("q"):
        break
         
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
  
# cleanup the camera and close any open windows
cv2.destroyAllWindows()
client.close()