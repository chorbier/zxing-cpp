#skip includes files in cp

sudo rm -r  v8/
sudo rm -r  v7/
sudo rm -r  x86/
sudo rm -r  x86_64/
sudo rm -r  release/

mkdir v8/
mkdir v7/
mkdir x86/
mkdir x86_64/
mkdir release/

cd release/

cmake .. -DCMAKE_TOOLCHAIN_FILE=/home/lev/Android/Sdk/ndk-bundle/android-ndk-r21/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_SHARED_LINKER_FLAGS="-llog"
make -j 16
sudo make install

sudo cp /usr/local/lib/libZXing.so ../v8/
file ../v8/libZXing.so

sudo rm -r *
sudo rm -rf /usr/local/lib/libZXing.so

cmake .. -DCMAKE_TOOLCHAIN_FILE=/home/lev/Android/Sdk/ndk-bundle/android-ndk-r21/build/cmake/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_SHARED_LINKER_FLAGS="-llog"
make -j 16
sudo make install


sudo cp /usr/local/lib/libZXing.so ../v7/
file ../v7/libZXing.so

sudo rm -r *
sudo rm -rf /usr/local/lib/libZXing.so

cmake .. -DCMAKE_TOOLCHAIN_FILE=/home/lev/Android/Sdk/ndk-bundle/android-ndk-r21/build/cmake/android.toolchain.cmake -DANDROID_ABI=x86 -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_SHARED_LINKER_FLAGS="-llog"
make -j 16
sudo make install

sudo cp /usr/local/lib/libZXing.so ../x86/
file ../x86/libZXing.so

sudo rm -r *
sudo rm -rf /usr/local/lib/libZXing.so

cmake .. -DCMAKE_TOOLCHAIN_FILE=/home/lev/Android/Sdk/ndk-bundle/android-ndk-r21/build/cmake/android.toolchain.cmake -DANDROID_ABI=x86_64 -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_SHARED_LINKER_FLAGS="-llog"
make -j 16
sudo make install

sudo cp /usr/local/lib/libZXing.so ../x86_64/
file ../x86_64/libZXing.so

sudo rm -r *
sudo rm -rf /usr/local/lib/libZXing.so
