cmake_minimum_required(VERSION 2.8.9)
project(pycheetah)
set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)



#execute_process(COMMAND ${CMAKE_SOURCE_DIR}/scripts/find_qt_path.sh OUTPUT_VARIABLE CMAKE_PREFIX_PATH_1)
#SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_1})
message("CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

#if(POLICY CMP0071)
	#	  cmake_policy(SET CMP0071 OLD)
	#endif()

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
get_target_property(QtCore_location Qt5::Core LOCATION)
message(STATUS ${QtCore_location})
find_package(Qt5Gui CONFIG REQUIRED)

set(QOBJECT_HEADERS
        include/Graphics3D.h
        include/GameController.h
        include/SimControlPanel.h
        include/JoystickTest.h)
#        include/main_window.h
#        include/gamepad_monitor.h
#        include/spi_debug.h)

QT5_WRAP_CPP(tst_hdr_moc ${QOBJECT_HEADERS})

include_directories("../sim/include/")
include_directories("../sim/src/")
include_directories("../common/include/")
include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../third-party/ParamHandler")
#file(GLOB sources "../sim/*.cpp")

include_directories("../user/MIT_Controller")
include_directories("../user/MIT_Controller/Controllers")
include_directories("../robot/include")
include_directories("../common/FootstepPlanner")
include_directories("../third-party/")
include_directories(SYSTEM "../third-party/qpOASES/include")
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories("..")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})


find_package(Qt5Core COMPONENTS QtGamepad REQUIRED)
find_package(Qt5Gamepad REQUIRED)

set(OpenGL_GL_PREFERENCE LEGACY)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(Eigen3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

#add_executable(pycheetah ${sources} ${tst_hdr_moc})
find_package(pybind11 CONFIG 2.3 REQUIRED)
pybind11_add_module(pycheetah bindings.cpp)

target_link_libraries(pycheetah PRIVATE biomimetics WBC_Ctrl NeuralMPC robot Goldfarb_Optimizer osqp dynacore_param_handler qpOASES mit_ctrl_lib
	#Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Gamepad 
	${QT_LIBRARIES} ${OPENGL_LIBRARIES})

install (TARGETS pycheetah DESTINATION lib)

#target_link_libraries(pycheetah biomimetics pthread lcm inih dynacore_param_handler 
#    Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Gamepad ${QT_LIBRARIES} ${OPENGL_LIBRARIES})

if (CMAKE_SYSTEM_NAME MATCHES Linux)
	target_link_libraries(pycheetah PRIVATE rt ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

#if (APPLE)
#  target_link_libraries(sim "-framework GLUT")
#endif (APPLE)

#add_subdirectory(pybind11)
#pybind11_add_module(pycheetah bindings.cpp)
