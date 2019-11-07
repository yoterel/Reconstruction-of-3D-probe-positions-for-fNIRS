# Video-based motion-resilient reconstruction of 3D position for fNIRS/EEG head mounted probes
## Users’ Guide
This application is designed to provide an accurate estimation of the position of an fNIRS probing cap on a participant’s head, based on a short video of the measurement process. It runs the entire processing pipeline, beginning in processing the video itself and concluding with producing a POS file with the cap’s position in MNI coordinates.
First, upon the application’s lunch, the user must provide several inputs in the start window (more details about the inputs further down). After clicking submit, the rest of the stages are executed sequentially. The flow of the application is roughly divided into the following stages:
1.	Load and process the video
2.	Run the VisualSFM tool on the edited video frames
3.	Load the generated .ply file (point cloud), match it to the model mesh and plot both
4.	Locate the coordinates of the stickers on the generated point cloud, and allow the user to mark the positions of the stickers that weren’t located (on top of the model mesh)
5.	Approximate the position of the cap on the participant’s head, convert the results to MNI coordinates in the POS file
### Application dependencies:
-	[x] A version of MATLAB which supports 2018b and 2019a code
-	[x] Visual SFM (installed together with pmvs to enable dense mesh reconstructions)
-	[x] The MATLAB SPM package
-	[x] The MATLAB SPM fNIRS package
- [ ] Python? This needs to be checked.
### Input to the application:
Upon launch, the application requires several input parameters:
-	Video path: the path to an .mp4 video file of the participant during the measurement process
-	Model mesh path: the path to a .ply file which is a mesh of the model of the cap, prepared ahead of time, and later on displayed to allow selecting missing points
-	Visual SFM path: path to the Visual SFM executable.
-	NIRS model path: path to a MATLAB file containing information about the NIRS model (such as NIRS_adult.mat)
-	MNI model path: path to a MATLAB file containing the positions of all key points on the head and probing cap (not just the stickers) in the model mesh. It can be generated from FixModelMNI.mat using the createMNIFileForModel.m script
-	Output directory: path to a directory in which all the output files (including intermediate files and the POS file) will be saved. It doesn’t have to be created prior to running the application
-	SPM path: path to the SPM installation directory
-	SPM FNIRS path: path to the SPM fNIRS installation directory
-	Shimadzu file path: path to the shimadzu text file (such as adult.txt)
-	Sticker HSV path: path to a file containing the HSV color of the stickers of the cap in the video (such as stickerHSV.txt)
## Advanced options for fine tuning:
-	Frame skip: the number of video frames to skip for each processed video frame. Default is 4
-	Sticker minimal group size: the minimal size of a cluster of points the generated point cloud for it to be considered as a separate sticker. Default is 5
-	Radius to sticker ratio: the (minimal) ratio between the radius of the sphere approximating the cap to the size of a sticker on the cap. As this value becomes larger, only smaller clusters will be considered to be separate stickers
## Developers’/Technical Users’ Guide
Creating an MNI.mat File for a New Cap
Each run of the application requires a file similar in structure to FixModelMNI.mat, which contains information about the locations of all key points on the probing cap. Since changing reconstructed model ply files might occur relatively frequently (especially if new caps are introduced), the project contains a script which allows generating the relevant *MNI.mat file automatically for the new ply. 
The script is called createMNIFileForModel.m and is directly in the root folder of the project. It works by taking a previously created *MNI.mat file and the new model ply as an input, locating the stickers in the new ply, and then calculating the rotation, translation and scaling between the model represented by the old .mat file to the one in the new ply file. These transformations are then applied to the provided *MNI.mat file to create the new file. 
The calculations performed in this script are based on the original plyToPOS.m script, but with several adaptations. In more detail, the script is composed of the following stages:
1.	Load the generated ply and get a point cloud composed only from the sticker candidates, based on the points’ hue
2.	Load the provided *MNI.mat file and extract the positions of the stickers in it
3.	Use approximating spheres for the sticker candidates and the model stickers to scale and translate all of the points from the *MNI.mat file
4.	Estimate the sticker positions in the ply files by calculating clusters and their centers
5.	Use the triplets-based matching between stickers the provided model stickers and the calculated ply stickers to find translation and rotation parameters
6.	Transform all of the model points, and use a maximum-weight-matching algorithm to label the stickers in the ply. Plot the two sets of stickers for comparison
7.	Use the coordinates of the transformed points to create the new file
The first few lines of the script need to be edited prior to running it. Specifically, the values for the following variables need to be edited:
-	mniModelPath: The path of the *MNI.mat file which will be the model for the new generated file
-	plyFileDir: the directory containing the new ply file
-	plyFileName: the name (with extension) of the new ply file
-	outputFileName: the name (with extension) of the output file, will be saved in the same folder as the ply file
-	stickerHSVPath: same as in the application
-	radiusToStickerRatio: same as in the application
-	stickerMinGroupSize: same as in the application
-	maxHueDiff: The maximum allowed difference between a point’s hue to the sticker’s hue (between 0 and 1)
A few more notes:
-	If not exactly 9 stickers are located in the provided ply, and error is thrown. The model ply is supposed to be clean and accurate enough so that the stickers can be properly located on it
-	The reason a non-default maxHueDiff parameter was added (used to be only 0.1) is because some of the tested model plys had a lot of green points that were purely noise and made locating the stickers problematic
Technical information regarding the application
The UI:
The application was developed using MATLAB’s GUIDE framework. MATLAB actually has a newer framework for developing GUIs called App Designer, but the problem is that it cannot be conveniently integrated into source control: the application is represented by a single binary file which contains the code itself, and therefore showing the difference in each commit cannot be done. GUIDE was a good enough solution, and in it, every binary file which represent a layout for a window also has a corresponding regular code file. If in the future it is decided that the look and feel of GUIDE apps isn’t good enough, the project can be converted into an App Designer application (MATLAB has tools for converting automatically).
More about the difference between the two frameworks can be seen here:[App Designer](https://uk.mathworks.com/products/matlab/app-designer/comparing-guide-and-app-designer.html), [MATLAB GUIDE](https://uk.mathworks.com/help/matlab/creating_guis/about-the-simple-guide-gui-example.html)
## More information about the application
-	If one wants to run the application without the part which creates the original ply, lines 96-103 in app.m (starting with data=load(…) and ending with plyFilePath =…) can be commented out, and the plyFilePath variable can be given a hard-coded value for a ply file on the computer. This can save a lot of time when working on features such as selecting points or IPC
-	Ply files generated by Visual SFM contain the properties diffuse_red/diffuse_green/diffuse_blue rather than the regular properties red/green/blue. However, MATLAB’s pointCloud object and other libraries/executbales (such as PoissonRecon.exe) expect the ply to contain the regular version. That’s why the function structToPointCloud was created. It should be combined with the function plyread which can read all of the .ply file’s properties, to create a pointCloud object with the regular color fields.
-	The file messing.m contains many demos and utility functions that I have used and experimented with throughout the development process. It might contain useful snippets for developing new features and debugging the application
- CapNet model file can be downloaded from ftp://anonymous@yotablog.com:1@ftp.yotablog.com/model.rar.
Notice this model was created using matlab 2018b, and is loadable only using a supporting matlab version (the model is a DAG network object).
In particular, "semanticseg" function must be supported in your version of matlab for the model to be usable.
