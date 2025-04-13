zxing_add_package_stb()

include (GNUInstallDirs)


#    add_test(NAME ZXingReaderTest COMMAND ZXingReader -fast -format qrcode test.png) # see above

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
    set(DEBUG_SOURCES
        DebugReadbardcode.cpp
        libs/UnwarpPreprocess.cpp
        libs/delaunator.cpp
    )
    add_executable (DebugReadbardcode ${DEBUG_SOURCES})
    target_include_directories (DebugReadbardcode PRIVATE libs)
    target_include_directories (DebugReadbardcode PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries (DebugReadbardcode ZXing::ZXing ${OpenCV_LIBS})
    # target_link_libraries (DebugReadbardcode ZXing::ZXing ${OpenCV_LIBS} DebugDrawStuff)
else ()
    message("INFO: OpenCV not found, skipping DebugReadbardcode")
endif()