#     Init
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
SET(KEYWORD video_processing)

# Start a project
PROJECT(drone)

#Find library
FIND_PACKAGE(OpenCV REQUIRED) #Need OpenCV
FIND_PACKAGE(Doxygen) #Pour la documentation
FIND_PACKAGE(Qt4 REQUIRED) #Need QT4


# Create bin folder depending on OS
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  ADD_DEFINITIONS(-DSYSTEMTYPE_LINUX)
    INCLUDE(${QT_USE_FILE})
    ADD_DEFINITIONS(${QT_DEFINITIONS})
	# SET system folder name
  SET(SYSTEM_FOLDER_NAME linux )
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	ADD_DEFINITIONS(-DSYSTEMTYPE_WINDOWS)
	# SET system folder name
	SET(SYSTEM_FOLDER_NAME windows)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#	ADD_DEFINITIONS(-DSYSTEMTYPE_MAC)
	# SET system folder name
	SET(SYSTEM_FOLDER_NAME darwin)	
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")


# SET variable
SET(MY_SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src)  #Variable src path
SET(MY_OUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}) #variable output path
SET(MY_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/out/lib) #variable lib path
SET(EXECUTABLE_OUTPUT_PATH ${MY_OUT_PATH}) #Variable Executable path
SET(LIBRARY_OUTPUT_PATH ${MY_LIB_PATH})

INCLUDE_DIRECTORIES(${MY_SRC_PATH})
INCLUDE_DIRECTORIES("${drone_BINARY_DIR}")

#Configuration de l'exécutable

file(

        GLOB_RECURSE

        source_files

        src/*

)

add_executable(

        my_exe

        ${source_files}

)



#Configuration de l'édition de liens

target_link_libraries(

        my_exe

        ${OpenCV_LIBS}

)

