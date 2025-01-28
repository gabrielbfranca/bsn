CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
PROJECT(component)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg messages archlib lepton libbsn)
find_package(GTest REQUIRED)
find_package(Boost REQUIRED COMPONENTS python${PYTHON_VERSION_MAJOR})
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)
###########################################################################
## ran an issue of gtest file directory not found, solved with:
#cd /usr/src/gtest
#sudo cmake .
#sudo make
#sudo cp *.a /usr/lib
## after that make sure to clean and build
##########################################################################
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS})


# Export catkin package.
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    LIBRARIES my_component
    CATKIN_DEPENDS messages message_runtime archlib lepton libbsn
)

# Set catkin directory.
INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})

# Set include directory.
INCLUDE_DIRECTORIES(include)
################################ TEST SETUP ###########################################
# Include test directory
include_directories(${GTEST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


#TARGET_LINK_LIBRARIES(run_test ${PROJECT_NAME} ${LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
# Link libraries to the test executable
# Add test executable
catkin_add_gtest(${PROJECT_NAME}_test
    #test/test_g4t1.cpp
    test/test_Sensor.cpp
)

if (TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test
        ${GTEST_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        archlib 
        lepton
        libbsn

    )
endif()
###########################################################################
# Build this project.
FILE(GLOB ${PROJECT_NAME}-src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/g3t1_3/*.cpp")
ADD_LIBRARY(${PROJECT_NAME} ${${PROJECT_NAME}-src})

SET(${PROJECT_NAME}-src
    ${${PROJECT_NAME}-src}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/g3t1_3/G3T1_3.cpp
)

SET(g3t1_1-src "${CMAKE_CURRENT_SOURCE_DIR}/src/g3t1_1/G3T1_1.cpp")
ADD_EXECUTABLE (g3t1_1  "${CMAKE_CURRENT_SOURCE_DIR}/apps/g3t1_1.cpp" ${${PROJECT_NAME}-src} ${g3t1_1-src})
TARGET_LINK_LIBRARIES (g3t1_1 ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(g3t1_1 messages_generate_messages_cpp)

SET(g3t1_2-src "${CMAKE_CURRENT_SOURCE_DIR}/src/g3t1_2/G3T1_2.cpp")
ADD_EXECUTABLE (g3t1_2  "${CMAKE_CURRENT_SOURCE_DIR}/apps/g3t1_2.cpp" ${${PROJECT_NAME}-src} ${g3t1_2-src})
TARGET_LINK_LIBRARIES (g3t1_2 ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(g3t1_2 messages_generate_messages_cpp)

SET(g3t1_3-src "${CMAKE_CURRENT_SOURCE_DIR}/src/g3t1_3/G3T1_3.cpp")
ADD_EXECUTABLE (g3t1_3  "${CMAKE_CURRENT_SOURCE_DIR}/apps/g3t1_3.cpp" ${${PROJECT_NAME}-src} ${g3t1_3-src})
TARGET_LINK_LIBRARIES (g3t1_3 ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(g3t1_3 messages_generate_messages_cpp)

SET(g3t1_4-src "${CMAKE_CURRENT_SOURCE_DIR}/src/g3t1_4/G3T1_4.cpp")
ADD_EXECUTABLE (g3t1_4  "${CMAKE_CURRENT_SOURCE_DIR}/apps/g3t1_4.cpp" ${${PROJECT_NAME}-src} ${g3t1_4-src})
TARGET_LINK_LIBRARIES (g3t1_4 ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(g3t1_4 messages_generate_messages_cpp)

SET(g3t1_5-src "${CMAKE_CURRENT_SOURCE_DIR}/src/g3t1_5/G3T1_5.cpp")
ADD_EXECUTABLE (g3t1_5  "${CMAKE_CURRENT_SOURCE_DIR}/apps/g3t1_5.cpp" ${${PROJECT_NAME}-src} ${g3t1_5-src})
TARGET_LINK_LIBRARIES (g3t1_5 ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(g3t1_5 messages_generate_messages_cpp)

SET(g3t1_6-src "${CMAKE_CURRENT_SOURCE_DIR}/src/g3t1_6/G3T1_6.cpp")
ADD_EXECUTABLE (g3t1_6  "${CMAKE_CURRENT_SOURCE_DIR}/apps/g3t1_6.cpp" ${${PROJECT_NAME}-src} ${g3t1_6-src})
TARGET_LINK_LIBRARIES (g3t1_6 ${catkin_LIBRARIES} ${LIBRARIES} ${Boost_LIBRARIES} ${Boost_PYTHON_LIBRARIES})
ADD_DEPENDENCIES(g3t1_6 messages_generate_messages_cpp)

SET(g4t1-src "${CMAKE_CURRENT_SOURCE_DIR}/src/g4t1/G4T1.cpp")
ADD_EXECUTABLE (g4t1 "${CMAKE_CURRENT_SOURCE_DIR}/apps/g4t1.cpp" ${${PROJECT_NAME}-src} ${g4t1-src})
TARGET_LINK_LIBRARIES (g4t1 ${catkin_LIBRARIES} ${LIBRARIES})
ADD_DEPENDENCIES(g4t1 messages_generate_messages_cpp)

# Set output properties to make it a Python module


add_library(my_component MODULE src/bindings.cpp)
target_link_libraries(my_component ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(my_component PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    #LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
###########################################################################
# Install this project.
catkin_install_python(PROGRAMS test/test_binding.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#INSTALL(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#INSTALL(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
