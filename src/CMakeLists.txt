#opencv
find_package (OpenCV REQUIRED)

#eigen3
set( EIGEN3_INCLUDE_DIR /usr/include/eigen3 )
if( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories ( "${EIGEN3_INCLUDE_DIR}" Apriltag Apriltag/common)


add_library(Apriltag Apriltag/apriltag.c Apriltag/apriltag_quad_thresh.c Apriltag/g2d.c Apriltag/getopt.c Apriltag/homography.c
        Apriltag/image_f32.c Apriltag/image_u8.c Apriltag/image_u8x3.c Apriltag/matd.c Apriltag/pnm.c Apriltag/string_util.c
        Apriltag/svd22.c Apriltag/tag16h5.c Apriltag/tag36artoolkit.c Apriltag/tag36h11.c Apriltag/time_util.c Apriltag/unionfind.c
        Apriltag/workerpool.c Apriltag/zarray.c Apriltag/zhash.c Apriltag/zmaxheap.c)
target_link_libraries(Apriltag)

add_library(FastSLAM fastslam/fastslam_core.cpp fastslam/utils.cpp)
target_link_libraries(FastSLAM)

add_library( tag
        tag.cpp taginfo.cpp kalman.cpp)
target_link_libraries( tag
        ${OpenCV_LIBS}
        Apriltag
        FastSLAM
        )


add_executable( solve solve.cpp parameterReader.cpp SLAMHelper.cpp )
target_link_libraries( solve
        tag
        ${OpenCV_LIBS}
        Apriltag
        lcm
        GCBase_gcc_v3_0_Basler_pylon_v5_0
        GenApi_gcc_v3_0_Basler_pylon_v5_0
        MathParser_gcc_v3_0_Basler_pylon_v5_0
        NodeMapData_gcc_v3_0_Basler_pylon_v5_0
        pylonbase-5.0.5
        XmlParser_gcc_v3_0_Basler_pylon_v5_0
        pylonutility-5.0.5
        Log_gcc_v3_0_Basler_pylon_v5_0
        bxapi-5.0.5
        gxapi-5.0.5
        log4cpp_gcc_v3_0_Basler_pylon_v5_0
        pylonc-5.0.5
        pylon_TL_gige-5.0.5
        uxapi-5.0.5
        )
