# Landmark Extractor
This python script extracts forehead location of faces from a video/image/image folder. The output consists of 3 points (x,y), one corrosponding to the left eye left most point, one to the right eye right most point and another of the tip of the nose. It uses the opencv DNN implementation of the "SSD: Single Shot MultiBox Detector" method.

The model file for face landmarks extraction be downloaded from [here](http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2).

The .m file is a matlab wrapper which uses the output of the python script and creates a polygon by reflecting the point of the nose about the line segment created by the eyes.

required python version:
3.4 and up

required python dependencies:
numpy
imutils
dlib
cv2
