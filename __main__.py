import cv2
import numpy as np
import os
import TrajectoryManager as tm


print(os.getcwd())





bRecord = False
if bRecord == True:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('{}.avi'.format(video_path.split('/')[-1].split('.')[0]),fourcc, 20.0, (640,360))
    out = cv2.VideoWriter("renderOutput.mp4", cv2.VideoWriter_fourcc('a', 'v', 'c', '1'), 20.0, (640,360))

cap = cv2.VideoCapture('video/videoplayback.mp4')
bgSubtractor = cv2.createBackgroundSubtractorKNN(history = 10, dist2Threshold = 200.0)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

kernel_size = 11
kernel_dilation = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))

frame_count = 0
trajectory_image = np.zeros([360, 640, 3], np.uint8)
point_image = np.zeros([360, 640, 3], np.uint8)


manager = tm.TrajectoryManager()


while cap.isOpened() :

    ret, frame = cap.read()
    resize_scale = 640. / float(frame.shape[1])

    frame = cv2.resize(frame, None, fx=resize_scale, fy=resize_scale)

    #print(frame.shape)


    # 가우시안 블러 적용
    blur = cv2.GaussianBlur(frame, (7, 7), 0)


    # Background 마스크 생성
    fgmask = bgSubtractor.apply(blur)
    blank_image = np.zeros(fgmask.shape, np.uint8)


    # Background 마스크에 모폴로지 적용
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_DILATE, kernel_dilation)



    point_image = cv2.addWeighted(point_image, 0.9, np.zeros(frame.shape, np.uint8), 0.1, 0)


    #print("frame_count :", frame_count)
    frame_count += 1

    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(fgmask)

    #print(len(centroids))



    points = []
    for index, centroid in enumerate(centroids):
        if stats[index][0] == 0 and stats[index][1] == 0:
            continue
        if np.any(np.isnan(centroid)):
            continue


        x, y, width, height, area = stats[index]
        centerX, centerY = int(centroid[0]), int(centroid[1])


        area_ratio = area / (width * height)
        aspect_ratio = width / height
        #print(x, y, area, width * height, area_ratio)



        #if area > 2 and area < 2000:
        if area_ratio > 0.6 and aspect_ratio > 0.333 and aspect_ratio < 3.0 and area < 500 and fgmask[centerY, centerX] == 255:

            #cv2.circle(frame, (centerX, centerY), 1, (0, 255, 0), 2)
            # cv2.rectangle(frame, (x-1, y-1), (x-1 + width+2, y-1 + height+2), (0, 0, 255))
            cv2.rectangle(frame, (x - 1, y - 1), (x - 1 + width + 2, y - 1 + height + 2), (0, 255, 0))

            point_image[centerY, centerX] = (255, 255, 255)
            points.append(np.array([centerY, centerX]))


            # 해당 포인트의 컬러 값 얻기
            for pixel_y in range(y, y + height):
                for pixel_x in range(x, x + width):

                    if fgmask[pixel_y, pixel_x] >= 0:
                        #frame[pixel_y, pixel_x] = [0, 255, 0]
                        blank_image[pixel_y, pixel_x] = 255

        #else :

         #   cv2.rectangle(frame, (x - 1, y - 1), (x - 1 + width + 2, y - 1 + height + 2), (0, 0, 255))


    manager.setPointsFrame(points)



    for trajectory in manager.getTrajectorys():

        points = trajectory.getPoints()

        #print(points)

        if len(points) < 3:
            continue


        for index, point in enumerate(points):
            if point[0] < 360 and point[1] < 640:
                trajectory_image[point[0], point[1]] = (0, 255, 0)
                cv2.circle(frame, (point[1], point[0]), 1, (255, 255, 0), 2)

                if index >= 1:
                    cv2.line(frame, (points[index-1][1], points[index-1][0]), (point[1], point[0]), (255, 255, 0), 1)



    #cv2.imshow('processed', fgmask)
    #cv2.imshow('point', point_image)
    cv2.imshow('raw', frame)

    # record
    if bRecord == True:
        out.write(frame)

    # terminate
    k = cv2.waitKey(4)
    if k == 27:
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        break


if bRecord == True:
    out.release()

cap.release()
cv2.destroyAllWindows()
