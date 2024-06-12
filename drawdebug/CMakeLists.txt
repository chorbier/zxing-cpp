cmake_minimum_required (VERSION 3.14)

zxing_add_package_stb()

include (GNUInstallDirs)


#    add_test(NAME ZXingReaderTest COMMAND ZXingReader -fast -format qrcode test.png) # see above

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
	add_library(DebugDrawStuff STATIC DebugDrawStuff.cpp)
    # target_compile_features(DebugDrawStuff PUBLIC cxx_std_17)
    target_include_directories (DebugDrawStuff PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_include_directories (DebugDrawStuff PUBLIC ${PROJECT_SOURCE_DIR}/drawdebug)
    # target_link_libraries (DebugDrawStuff INTERFACE ZXing::ZXing)
    target_link_libraries (DebugDrawStuff INTERFACE ${OpenCV_LIBS})

    target_link_libraries (ZXing PRIVATE DebugDrawStuff)
    
    set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fPIC" )

else ()
    message("INFO: OpenCV not found, skipping DebugReadbardcode")
endif()