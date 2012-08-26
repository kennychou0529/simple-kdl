rem "build SimpleKDL 64bit"
mkdir build64
cd build64
cmake -G "Visual Studio 9 2008 Win64" ^
-DCMAKE_BUILD_TYPE=Release ^
-DEIGEN3D_INCLUDE="C:\Users\Public\Documents\development\libs\graphics\3d\eigen\eigen-eigen-c40708b9088d" ^
-DBOOST_ROOT="C:\Users\Public\Documents\development\libs\os\boost\boost_1_46_1" ^
..
cd ..