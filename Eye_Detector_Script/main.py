import cv2
import dlib
import numpy as np
from scipy.spatial import distance as dist
import os
current_path = os.path.dirname(os.path.abspath(__file__))
PREDICTOR_PATH = os.path.join(current_path,"shape_predictor_68_face_landmarks.dat")

EYE_AR_THRESH = 0.20
EYE_AR_CONSEC_FRAMES = 3

# initialize dlib’s face detector and shape predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(PREDICTOR_PATH)

# index slices for the left and right eye landmarks
(lStart, lEnd) = (42, 48)
(rStart, rEnd) = (36, 42)

def eye_aspect_ratio(eye):
    # compute the euclidean distances between the two sets of vertical eye landmarks
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])
    # compute the euclidean distance between the horizontal landmarks
    C = dist.euclidean(eye[0], eye[3])
    # compute eye aspect ratio
    ear = (A + B) / (2.0 * C)
    return ear

def main():
    cap = cv2.VideoCapture(0)
    blink_counter = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects = detector(gray, 0)

        for rect in rects:
            shape = predictor(gray, rect)
            coords = np.zeros((68, 2), dtype="int")
            for i in range(0, 68):
                coords[i] = (shape.part(i).x, shape.part(i).y)

            leftEye = coords[lStart:lEnd]
            rightEye = coords[rStart:rEnd]
            leftEAR = eye_aspect_ratio(leftEye)
            rightEAR = eye_aspect_ratio(rightEye)
            ear = (leftEAR + rightEAR) / 2.0

            # draw eye contours
            leftHull = cv2.convexHull(leftEye)
            rightHull = cv2.convexHull(rightEye)
            cv2.drawContours(frame, [leftHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [rightHull], -1, (0, 255, 0), 1)

            # check if eye is closed
            status = "Open"
            if ear < EYE_AR_THRESH:
                blink_counter += 1
                if blink_counter >= EYE_AR_CONSEC_FRAMES:
                    status = "Closed"
            else:
                blink_counter = 0

            # display status and EAR
            cv2.putText(frame, f"EAR: {ear:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"Eye: {status}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow("Eye Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()