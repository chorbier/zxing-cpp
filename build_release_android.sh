#skip includes files in cp

CMAKE_TOOLCHAIN_FILE="/home/chorbier/Android/Sdk/ndk-bundle/android-ndk-r21/build/cmake/android.toolchain.cmake"

# Parse named arguments
for arg in "$@"; do
  case $arg in
    CMAKE_TOOLCHAIN_FILE=*)
      CMAKE_TOOLCHAIN_FILE="${arg#*=}"
      ;;
  esac
done

# sudo rm -rf zxing-cpp.release
cd zxing-cpp

sudo rm -r ../zxing-cpp/arm64-v8a/
sudo rm -r ../zxing-cpp/armeabi-v7a/
sudo rm -r ../zxing-cpp/x86/
sudo rm -r ../zxing-cpp/x86_64/
sudo rm -r ../zxing-cpp/release/


mkdir arm64-v8a/
mkdir armeabi-v7a/
mkdir x86/
mkdir x86_64/
mkdir release/

cd release/

OPENCV_PATH="~/opencv_android_4.3"
# OPENCV_PATH="~/opencv_android_3.4.3"
OPENCV_JNI_PATH="$OPENCV_PATH/sdk/native/jni"

LINKER_FLAGS="-llog -Wl,--strip-all"

cmake .. -DCMAKE_TOOLCHAIN_FILE="$CMAKE_TOOLCHAIN_FILE" -DOpenCV_DIR="$OPENCV_JNI_PATH" -DANDROID_ABI=arm64-v8a -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_SHARED_LINKER_FLAGS="$LINKER_FLAGS" -DBUILD_FOR_AARM=ON
make -j 16

cp core/libZXing.so ../arm64-v8a/
file ../arm64-v8a/libZXing.so
sudo rm -r ../../zxing-cpp/release/*



cmake .. -DCMAKE_TOOLCHAIN_FILE="$CMAKE_TOOLCHAIN_FILE" -DOpenCV_DIR="$OPENCV_JNI_PATH" -DANDROID_ABI=armeabi-v7a -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_SHARED_LINKER_FLAGS="$LINKER_FLAGS" -DBUILD_FOR_AARM=ON
make -j 16

cp core/libZXing.so ../armeabi-v7a/
file ../armeabi-v7a/libZXing.so
sudo rm -r ../../zxing-cpp/release/*



cmake .. -DCMAKE_TOOLCHAIN_FILE="$CMAKE_TOOLCHAIN_FILE" -DOpenCV_DIR="$OPENCV_JNI_PATH" -DANDROID_ABI=x86 -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_SHARED_LINKER_FLAGS="$LINKER_FLAGS" -DBUILD_FOR_AARM=ON
make -j 16

cp core/libZXing.so ../x86/
file ../x86/libZXing.so
sudo rm -r ../../zxing-cpp/release/*



cmake .. -DCMAKE_TOOLCHAIN_FILE="$CMAKE_TOOLCHAIN_FILE" -DOpenCV_DIR="$OPENCV_JNI_PATH" -DANDROID_ABI=x86_64 -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_SHARED_LINKER_FLAGS="$LINKER_FLAGS" -DBUILD_FOR_AARM=ON
make -j 16

cp core/libZXing.so ../x86_64/
file ../x86_64/libZXing.so
sudo rm -r ../../zxing-cpp/release/*
