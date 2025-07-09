zxing_add_package_stb()

include (GNUInstallDirs)


#    add_test(NAME ZXingReaderTest COMMAND ZXingReader -fast -format qrcode test.png) # see above

# option(WITH_PROFILING "Enable gperftools profiling" ON)
find_library(Gperftools REQUIRED)
message(STATUS "Profiling enabled (gperftools)")

# if(WITH_PROFILING)
#     find_library(Gperftools REQUIRED)
#     message(STATUS "Profiling enabled (gperftools)")
# else()
#     message(STATUS "Profiling disabled")
# endif()

# if(WITH_PROFILING AND TARGET Gperftools::profiler)
#     target_link_libraries(DebugReadbardcode PRIVATE Gperftools::profiler)
# endif()

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
    target_link_libraries (DebugReadbardcode ZXing::ZXing ${OpenCV_LIBS} profiler)
    # target_link_libraries(DebugReadbardcode ZXing::ZXing Gperftools::profiler)
    # target_link_libraries (DebugReadbardcode ZXing::ZXing ${OpenCV_LIBS} DebugDrawStuff)
else ()
    message("INFO: OpenCV not found, skipping DebugReadbardcode")
endif()