cmake_minimum_required(VERSION 3.16)
project(FOSU_AWAKENLION)

set(CMAKE_CXX_STANDARD 14)
set(ngraph_DIR "/usr/local/runtime/cmake")
set(InferenceEngine_DIR "/usr/local/runtime/cmake")

find_package(OpenCV 4 REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        #OpenVINO推理引擎的头文件
        /usr/local/runtime/include/ie/
        /usr/local/runtime/include/ngraph/
        /usr/local/runtime/include/openvino/

        ${OpenVINO_LIBRARY_DIRS}
        ${OpenCV_LIBRARY_DIRS}
        /lib/

        /home/rm/Infantry_2023.7.21_test
        

)

set(SOURCES
Global_user/global_user.cpp
Settings/Settings.cpp
RuneDetector/inference_api.cpp
Serial/Serial.cpp
Serial/Protocol.cpp
AngleSolver/PnpSolver.cpp
Serial/PackData.cpp
Serial/InfantryInfo.cpp
main.cpp
Camera/MVVideoCapture.cpp
ArmorDetector/ArmorDetector.cpp
ArmorDetector/inference_api2.cpp
kalman/armor_kalman.cpp
predict/predict.cpp
)

set(HEADERS
Global_user/global_user.hpp
Settings/Settings.h
RuneDetector/inference_api.hpp
Serial/Protocol.h
AngleSolver/PnpSolver.h
Serial/PackData.h
Serial/JudgementInfo.h
Serial/InfantryInfo.h
ArmorDetector/ArmorDetector.h
ArmorDetector/inference_api2.hpp
Camera/CameraApi.h
Camera/MVVideoCapture.h
kalman/armor_kalman.h
Serial/Serial.h
predict/predict.h

)

set(DISTFILES
 config_file/param_rune.yml
 config_file/param_other.yml
 config_file/param_armor.yml
 config_file/param_throw_compensate.yml
 config_file/param_pid-sentry7.yml
 config_file/param_pid-hero1.yml
 config_file/param_pid-fantry5.yml
 config_file/param_pid-fantry3.yml
 config_file/param_kalman.yml
 calibration/Camera752-hero1.xml
 config_file/param_kalman.yml
 calibration/Camera752-infantry.xml
)
add_executable(FOSU_AWAKENLION ${SOURCES} ${HEADERS} ${DISTFILES})

target_link_libraries(
        FOSU_AWAKENLION
        ${OpenCV_LIBS}
        ${InferenceEngine_LIBRARIES}
        -pthread
        libMVSDK.so
)

#add_executable(FOSU_AWAKENLION )
#target_link_libraries(
#        FOSU_AWAKENLION
#)
