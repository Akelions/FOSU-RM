QT += core
QT -= gui

CONFIG += c++11

TARGET = FOSU_AWAKENLION_2023
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += -I/usr/local/include \
               -I/usr/local/include/opencv \
               -I/usr/local/include/opencv2 \
               -I/usr/local/runtime/include/openvino \
               -I/usr/local/runtime/include/ngraph \
               -I/usr/local/runtime/include/ie \

LIBS += -L/usr/local/lib -lopencv_calib3d -lopencv_core -lopencv_dnn -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video -lopencv_videoio
LIBS += `pkg-config --libs opencv` -lMVSDK
LIBS += -L/usr/local/runtime/lib/intel64 -lgna -lopenvino -lopenvino_auto_batch_plugin -lopenvino_auto_plugin -lopenvino_c -lopenvino_gapi_preproc -lopenvino_hetero_plugin -lopenvino_intel_cpu_plugin -lopenvino_intel_gna_plugin -lopenvino_intel_gpu_plugin -lopenvino_intel_myriad_plugin -lopenvino_ir_frontend -lopenvino_onnx_frontend -lopenvino_paddle_frontend
INCLUDEPATH = -I./include

HEADERS += \
#    include/AttackTop.h \
#    include/VideoCaptureFactory.h \
#    include/Tool.h \
    ArmorDetector/TargetDetection.h \
#    include/ShapeGan.h \
    Settings/Settings.h \
    Serial/Serial.h \
    RuneDetector/RuneDetector.h \
    Serial/Protocol.h \
    Preprocessing/Preprocessing.h \
    AngleSolver/PnpSolver.h \
    Serial/PackData.h \
#    include/Kalman.h \
    Serial/JudgementInfo.h \
    Serial/InfantryInfo.h \
#    include/IndVideoCapture.h \
    Gui/Gui.h \
#    include/Common.h \
#    include/ColorDetector.h \
    ArmorDetector/ArmorDetector.h \
#    include/CameraStatus.h \
#    include/CameraDefine.h \
    Camera/CameraApi.h \
    Camera/MVVideoCapture.h \
#    include/VisualPid.h \
#    include/CCalibration.h \
#    include/DigitDetector.h \
#    include/Predictor.h \
#    include/VideoControl.h \
#    include/FFCSolver.h \
#    include/ExtendedKalmanFilter.h \
    AngleSolver/GravityCompensateResolve.h \
#    include/Engineer.h \
    AngleSolver/bayesEstimateVelocity.h \
#    include/RMVideoCapture.h \
    svm/svm.h \
    kalman/armor_kalman.h \
    kalman/armor_kalman.h \
    kalman/armor_kalman.h \
    kalman/Rune_kalman.h \
    Global_user/global_user.hpp \
    ArmorDetector/inference_api2.hpp \
    predict.h

SOURCES += \
#    src/VideoCaptureFactory.cpp \
#    src/Tool.cpp \
    ArmorDetector/TargetDetection.cpp \
#    src/ShapeGan.cpp \
    Settings/Settings.cpp \
    Serial/Serial.cpp \
    RuneDetector/RuneDetector.cpp \
#    src/RMVideoCapture.cpp \
    Serial/Protocol.cpp \
    Preprocessing/Preprocessing.cpp \
    AngleSolver/PnpSolver.cpp \
    Serial/PackData.cpp \
    Serial/InfantryInfo.cpp \
    main.cpp \
    Camera/MVVideoCapture.cpp \
#    src/IndVideoCapture.cpp \
    Gui/Gui.cpp \
#    src/ColorDetector.cpp \
    ArmorDetector/ArmorDetector.cpp \
#    src/MVVideoCapture.cpp \
#    src/Predictor.cpp \
#    src/DigitDetector.cpp \
#    src/VideoControl.cpp \
    AngleSolver/GravityCompensateResolve.cpp \
    AngleSolver/bayesEstimateVelocity.cpp \
    svm/svm.cpp \
    kalman/armor_kalman.cpp \
    kalman/Rune_kalman.cpp \
    Global_user/global_user.cpp \
    ArmorDetector/inference_api2.cpp \
    predict.cpp

DISTFILES += \
    config_file/param_rune.yml \
    config_file/param_other.yml \
    config_file/param_armor.yml \
    config_file/param_throw_compensate.yml \
    calibration/Camera640.xml \
    config_file/param_pid-sentry7.yml \
    config_file/param_pid-hero1.yml \
    config_file/param_pid-fantry5.yml \
    config_file/param_pid-fantry3.yml \
    config_file/right.jpg \
    config_file/left.jpg \
    src/README.md \
    calibration/default.xml \
    config_file/param_kalman.yml \
    calibration/Camera752-hero1.xml \
    config_file/param_kalman.yml \
    calibration/Camera752-infantry.xml
