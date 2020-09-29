# Tennis_Ball_Tracking-python

Python과 OpenCV를 이용한 영상에서 테니스 공 궤적 인식


## 결과물
![스크린샷 2019-01-10 오후 10 21 15](https://user-images.githubusercontent.com/7419790/94501281-763dff80-023c-11eb-92b5-d890ddefd58e.png)


## 코드 설명
- 공 후보를 나타내는 바이너리 이미지 생성
Gaussian Blur → Background Subtraction → Morphology (Open) → Morphology (Dilate)

- 공 후보의 위치와 크기 구하기
connectedComponentsWithStats

- 시간 흐름에 따른 Trajectory 추출
포인트를 하나씩 추가하며 마지막 3개의 포인트가 특정 조건을 만족하는지 확인
만족하는 경우 Trajectory 포인트 배열 확장 / 만족 못하면 Trajectory 종료
