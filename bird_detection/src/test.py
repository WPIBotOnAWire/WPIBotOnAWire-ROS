import cv2

vc = cv2.VideoCapture(0)

if vc.isOpened():
        rval, frame = vc.read()

else:
    rval = False

while rval:
      cv2.imshow("preview", frame)
      rval, frame = vc.read()
      key = cv2.waitKey(20)

vc.release()
cv2.destroyWindow("preview")