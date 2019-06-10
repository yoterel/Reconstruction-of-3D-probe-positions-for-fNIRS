import numpy as np
from imutils import face_utils
import dlib
import argparse
import cv2
import sys
import os
import glob

def get_landmarks(frame):
	landmarks=[]
	rects = detector(frame, 0)
	for (i, rect) in enumerate(rects):
		# Make the prediction and transform it to numpy array
		shape = predictor(frame, rect)
		shape = face_utils.shape_to_np(shape)
		landmarks.append([shape[30,0].item(), shape[30,1].item(), shape[17,0].item(),shape[17,1].item(), shape[26,0].item(), shape[26,1].item()])
	return landmarks

if __name__ == "__main__" :
	parser = argparse.ArgumentParser()
	group = parser.add_mutually_exclusive_group(required=True)
	group.add_argument('--image_folder', help="path to .png file to extract face bboxes from.")
	group.add_argument('--image_file', help="path to folder with .png images - will be processed lexicographically.")
	group.add_argument('--video_file', help="the input video file to extract face bboxes from")
	parser.add_argument("-f",'--image_format', default='png', help="specify the image format to be read as a 3 letter extension (default: png)")
	parser.add_argument("model", help="path to model file")
	parser.add_argument("-s","--skip_frames", type=int, help="skip every x number of frames from the video (0 = no skip, 1 = get every even frame, etc.)")
	if len(sys.argv) == 1:
		parser.print_help(sys.stderr)
		sys.exit(1)
	args = parser.parse_args()
	print('cv2 version:', cv2.__version__)
	detector = dlib.get_frontal_face_detector()
	predictor = dlib.shape_predictor(args.model)
	
	landmarks=[]
	landmarks_from_frame=[]
	if args.image_file is not None:
		image = cv2.imread(args.image_file, cv2.IMREAD_GRAYSCALE)
		landmarks_from_frame = get_landmarks(image)
		landmarks.append(landmarks_from_frame)
	else:
		if args.image_folder is not None:
			dir=args.image_folder
			print("Ignoring skip frames argument since folder_as_input option is true")
			query="*."+args.image_format
			file_counter = len(glob.glob1(dir,query))
			print("Found:", file_counter, "files ending with", args.image_format) 
			for root, dirs, files in os.walk(dir):
				for file in sorted(files):
					if file.endswith("."+args.image_format):
						print('Processing: ', os.path.join(root, file))
						image = cv2.imread(os.path.join(root, file),cv2.IMREAD_GRAYSCALE)
						landmarks_from_frame = get_landmarks(image)
						landmarks.append(landmarks_from_frame)
		else:
			vidcap = cv2.VideoCapture(args.input_file)
			length = int(vidcap.get(cv2.CAP_PROP_FRAME_COUNT))
			success, image = vidcap.read()
			count = 0
			print('Read a new frame: ', count, '/' , length)
			while success==True:
				landmarks_from_frame = get_landmarks(image)
				landmarks.append(landmarks_from_frame)
				if args.skip_frames is not None:
					for i in range(args.skip_frames+1):
						success,image = vidcap.read()
						count += 1
				else:
					success,image = vidcap.read()
					count += 1
				print('Read a new frame: ', count, '/' , length)
				sys.stdout.flush()
	print(landmarks)
	