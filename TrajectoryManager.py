import numpy as np


class Trajectory:

    def __init__(self, point):
        self.__points = []  # points 의 각 요소는 [2] shape의 numpy 배열이어야 함
        self.__length = 0
        self.__missing_count = 0

        self.__points.append(point)



    # Trajectory 포인트를 리턴하는 함수
    def getPoints(self):
        return self.__points


    # Trajectory 포인트를 추가하는 함수
    def addPoint(self, point):
        self.__points.append(point)
        self.__missing_count = 0

        last_points = self.__points[-2:]
        point_diff = last_points[1] - last_points[0]
        point_diff_mag = np.sqrt(point_diff.dot(point_diff))
        self.__length += point_diff_mag


    def getLength(self):
        return self.__length



    def checkNextPoint(self, point):

        points_length = len(self.__points)

        if points_length >= 3:
            # points 배열에 3개 이상의 포인트가 있는 경우
            # 다음 예측 포인트와 거리 확인
            #nextPoint = Trajectory.predictNextPoint(self.__points)
            #point_diff = point - nextPoint
            #point_diff_mag = np.sqrt(point_diff.dot(point_diff))
            #return (point_diff_mag < 8.0)
            return Trajectory.checkTriplet(self.__points[-2:] + [point])

        elif points_length == 0:
            # points 배열이 비어있는 경우
            # 무조건 True 리턴
            return True

        elif points_length == 1:
            # points 배열에 1개의 포인트만 있는 경우
            # 거리만 확인
            point_diff = point - self.__points[0]
            point_diff_mag = np.sqrt(point_diff.dot(point_diff))
            return (point_diff_mag > 2.0) and (point_diff_mag < 80.0)

        elif points_length == 2:
            # points 배열에 2개의 포인트가 있는 경우
            # Triplet 여부 확인
            return Trajectory.checkTriplet(self.__points + [point])



    # Missing 카운트 올리는 함수 -> 추적 계속 여부 리턴
    def upcountMissing(self):

        if len(self.__points) < 3:
            return False

        self.__missing_count += 1

        # missing count 초과 여부 확인
        if self.__missing_count > 1:
            # 추적 종료
            return False

        else:
            # 다음 추정 포인트 추가
            nextPoint = self.predictNextPoint(self.__points)
            self.__points.append(nextPoint)
            return True




    # 다음 포인트를 예측하여 리턴하는 함수
    @classmethod
    def predictNextPoint(self, points):

        if len(points) >= 3 :

            # 뒤에서 3개 포인트 추출
            last3Points = points[-3:]

            # 속도와 가속도 구함
            velocity = [last3Points[1] - last3Points[0], last3Points[2] - last3Points[1]]
            acceleration = velocity[1] - velocity[0]

            # 다음 위치 추정
            nextVelocity = velocity[1] + acceleration
            nextPoint = last3Points[2] + nextVelocity

            return nextPoint



    # 초기 유효 3포인트 만족 여부를 확인하는 함
    @classmethod
    def checkTriplet(self, points):

        if len(points) != 3:
            return False

        # 속도와 가속도 구함
        velocity = [points[1] - points[0], points[2] - points[1]]
        acceleration = velocity[1] - velocity[0]

        #print("acceleration :", acceleration)

        # 속도 크기가 비슷해야 함
        velocity_mag = [np.sqrt(velocity[0].dot(velocity[0])), np.sqrt(velocity[1].dot(velocity[1]))]
        if velocity_mag[0] > velocity_mag[1]:
            if velocity_mag[1] / velocity_mag[0] < 0.6:
                #print("velocity_mag[1] / velocity_mag[0] :", velocity_mag[1] / velocity_mag[0])
                return False
        else:
            if velocity_mag[0] / velocity_mag[1] < 0.6:
                #print("velocity_mag[0] / velocity_mag[1] :", velocity_mag[0] / velocity_mag[1])
                return False

        # 속도가 너무 작거나 크지 않아야 함
        if velocity_mag[0] < 2.0 or velocity_mag[0] > 80.0:
            #print("velocity_mag[0] :", velocity_mag[0])
            return False
        if velocity_mag[1] < 2.0 or velocity_mag[1] > 80.0:
            #print("velocity_mag[1] :", velocity_mag[1])
            return False

        # 속도 방향 변화가 작아야 함
        velocity_dot = velocity[1].dot(velocity[0])
        acceleration_angle = np.arccos(velocity_dot / (velocity_mag[0] * velocity_mag[1]))
        #print("acceleration_angle :",  np.rad2deg(acceleration_angle))
        if acceleration_angle > np.deg2rad(45.0):
            return False

        # 가속도가 작아야 함
        acceleration_mag = np.sqrt(acceleration.dot(acceleration))
        if acceleration_mag > 20.0:
            #print("acceleration_mag :", acceleration_mag)
            return False

        if acceleration[0] < -2.0:
            return False

        return True









class TrajectoryManager:

    def __init__(self):
        self.__trajectorys = []


    def getTrajectorys(self):
        return self.__trajectorys


    def setPointsFrame(self, points):

        max_trajectory = (0, 0)
        trajectorys_updated = [False] * len(self.__trajectorys)

        for index, point in enumerate(points):

            isAddedTrajectory = False

            # 기존 Trajectory에 추가되는 포인트인지 확인
            for index, updated in enumerate(trajectorys_updated):

                if updated == False:
                    if self.__trajectorys[index].checkNextPoint(point):
                        self.__trajectorys[index].addPoint(point)
                        trajectorys_updated[index] = True
                        isAddedTrajectory = True

                        trajectory_length = self.__trajectorys[index].getLength()
                        if trajectory_length > max_trajectory[0]:
                            max_trajectory = (trajectory_length, index)

                        break


            # Trajectory에 추가되지 않은 포인트는 신규 Trajectory로 생성
            if isAddedTrajectory == False:
                trajectory_new = Trajectory(point)
                self.__trajectorys.append(trajectory_new)


        # 높은 가능성의 Trajectory가 찾아지면 해당 Trajectory만 남김
        if max_trajectory[0] > 30.0:
            self.__trajectorys = [self.__trajectorys[max_trajectory[1]]]

        else:

            # 업데이트 되지 않은 Trajectory의 Missing Count 증가
            for index, updated in reversed(list(enumerate(trajectorys_updated))):

                if updated == False:
                    if self.__trajectorys[index].upcountMissing() == False:
                        self.__trajectorys.remove(self.__trajectorys[index])




