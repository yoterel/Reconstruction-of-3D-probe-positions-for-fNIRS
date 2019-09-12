:: Intended to be used with the PoissonRecon.exe file obtained from 
:: http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version12.00/
:: If color extraction is needed, the ply file must first be converted using the structToPointCloud
:: function, and then saved as a new .ply file. In such a case, simply add the --colors flag
SET EXE_DIR=C:\TEMP
SET INPUT_PLY=C:\TEMP\SagiUpdatedAdultCleaned-1.4-3.ply
SET OUTPUT_PLY=C:\TEMP\TempReconstructed.ply
%EXE_DIR%\PoissonRecon.exe --in %INPUT_PLY% --out %OUTPUT_PLY% --normals --samplesPerNode 5
PAUSE