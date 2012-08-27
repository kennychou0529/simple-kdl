# -----------------------------------------------------------------------------
# SimpleKDL
# -----------------------------------------------------------------------------
# Processing Wrapper for the KDL - Kinematic Dynamics Library
# http://code.google.com/p/simple-kdl
# --------------------------------------------------------------------------
# prog:  Max Rheiner / Interaction Design / zhdk / http://iad.zhdk.ch/
# date:  08/12/2012 (m/d/y)
# -----------------------------------------------------------------------------
#
#	this envVar should be set:
#		JAVA_HOME
#
# -----------------------------------------------------------------------------
	
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(CheckIncludeFiles)

PROJECT(SimpleKDL)

IF(NOT MACH_ARCH)
    # default 64bit
    SET(MACH_ARCH "64")
ENDIF()

# -----------------------------------------------------------------------------
# check swig
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})

# -----------------------------------------------------------------------------
# check java/jni
FIND_PACKAGE(Java REQUIRED)
FIND_PACKAGE(JNI REQUIRED)
INCLUDE_DIRECTORIES(${JAVA_INCLUDE_PATH})
INCLUDE_DIRECTORIES(${JAVA_INCLUDE_PATH2})
 
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
 
# -----------------------------------------------------------------------------
# java tools
IF(WIN32)
	SET(JAVA_RUNTIME2  $ENV{JAVA_HOME}/bin/java)
	SET(JAVA_ARCHIVE2  $ENV{JAVA_HOME}/bin/jar)
	SET(JAVA_COMPILE2  $ENV{JAVA_HOME}/bin/javac)
	SET(JAVA_DOC	   $ENV{JAVA_HOME}/bin/javadoc)
ELSE()
	# unix
	SET(JAVA_RUNTIME2  java)
	SET(JAVA_ARCHIVE2  jar)
	SET(JAVA_COMPILE2  javac)
	SET(JAVA_DOC	   javadoc)
ENDIF()


# -----------------------------------------------------------------------------
# boost
FIND_PACKAGE(Boost REQUIRED)
IF(Boost_INCLUDE_DIRS MATCHES "Boost_INCLUDE_DIR-NOTFOUND")
	MESSAGE("Boost should be set, for example:")
	MESSAGE("> cmake -DBOOST_ROOT=/usr/??? ..")	
ELSE()
	INCLUDE_DIRECTORIES ( ${Boost_INCLUDE_DIRS} )
	LINK_DIRECTORIES ( ${Boost_LIBRARY_DIRS} )
	message(${Boost_INCLUDE_DIRS})
	message(${Boost_LIBRARY_DIRS})
  
	IF(APPLE)
		# osx
		set(Boost_LIBRARIES  "boost_thread.a")
	ELSEIF(UNIX)
		# unix/linux,shared libs of boost should be installed
		set(Boost_LIBRARIES  "boost_thread.a")
	ELSE()
		# windows will find the right libs by itself
	ENDIF()
ENDIF()

# -----------------------------------------------------------------------------
# orocos - kdl

SET(LIBS ${LIBS} orocos-kdl)

# -----------------------------------------------------------------------------
# swig 
SET(CMAKE_SWIG_FLAGS -package SimpleKDL)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

# -----------------------------------------------------------------------------
# set local libraries
LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# load from local folder
SET(CMAKE_SKIP_RPATH NO)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
SET(CMAKE_INSTALL_RPATH "\$ORIGIN")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

# -----------------------------------------------------------------------------
# processing wrappers
SET(P5_WRAPPER ${CMAKE_CURRENT_SOURCE_DIR}/src/p5_src/)
SET(P5_WRAPPER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/p5_src/SimpleKDLContext.java)

# processing core lib
# shoud use find_program
IF(WIN32)
	SET(P5_JAR "C:/Program Files (x86)/processing-1.2.1/lib/core.jar")	
ELSEIF(APPLE)
	SET(P5_JAR "/Applications/Processing.app/Contents/Resources/Java/core.jar")
ELSE()
	# unix/linux
	# SET(P5_JAR "/?")
	IF(NOT DEFINED P5_JAR)
		MESSAGE("Set P5_JAR to the folder where Processing installed the core.jar file, for example:")
		MESSAGE(FATAL_ERROR "> cmake -DP5_JAR=/usr/???/core.jar ..")	
	ENDIF()
ENDIF()

IF(WIN32)
        SET(P5_JAR_OPENGL "C:/Program Files (x86)/processing-1.2.1/modes/java/libraries/opengl/library/opengl.jar")
ELSEIF(APPLE)
        SET(P5_JAR_OPENGL "/Applications/Processing.app/Contents/Resources/Java/modes/java/libraries/opengl/library/opengl.jar")
ELSE()
        # unix/linux
        # SET(P5_JAR_OPENGL "/?")
        IF(NOT DEFINED P5_JAR_OPENGL)
                MESSAGE("Set P5_JAR_OPENGL to the folder where Processing installed the opengl.jar file, for example:")
                MESSAGE(FATAL_ERROR "> cmake -DP5_JAR=/usr/???/opengl.jar.zip ..")
        ENDIF()
ENDIF()

IF(WIN32)
        SET(P5_JAR_JOGL  "C:/Program Files (x86)/processing-1.2.1/modes/java/libraries/opengl/library/jogl.jar")
ELSEIF(APPLE)
        SET(P5_JAR_JOGL "/Applications/Processing.app/Contents/Resources/Java/modes/java/libraries/opengl/library/jogl.jar")
ELSE()
        # unix/linux
        # SET(P5_JAR_JOGL "/?")
        IF(NOT DEFINED P5_JAR_JOGL)
                MESSAGE("Set P5_JAR_JOGL to the folder where Java Opengl installed the jogl.jar file, for example:")
                MESSAGE(FATAL_ERROR "> cmake -DP5_JAR=/usr/???/jogl.jar.zip ..")
        ENDIF()
ENDIF()

# -----------------------------------------------------------------------------
# eigen 3d template lib
IF(DEFINED EIGEN3D_INCLUDE)
	INCLUDE_DIRECTORIES(${EIGEN3D_INCLUDE})
ELSE()
	# check if the envvar is set
	SET(EIGEN3D_INCLUDE $ENV{EIGEN3D_INCLUDE})
	IF(DEFINED EIGEN3D_INCLUDE)
		INCLUDE_DIRECTORIES(${EIGEN3D_INCLUDE})
	ELSE()
		MESSAGE("EIGEN3D_INCLUDE should be set, for example:")
		MESSAGE("> cmake -DEIGEN3D_INCLUDE=/usr/??? ..")	
	ENDIF()
ENDIF()	


SET(SWIG_SOURCES 
src/SimpleKDLMain.i
src/RayIntersection.cpp
)

SET_SOURCE_FILES_PROPERTIES(${SWIG_SOURCES} PROPERTIES CPLUSPLUS ON)

# set the folder where the swig files should land
SET(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/swig_src)
MAKE_DIRECTORY(${CMAKE_CURRENT_BINARY_DIR}/src)


SWIG_ADD_MODULE(SimpleKDL Java ${SWIG_SOURCES})
SWIG_LINK_LIBRARIES(SimpleKDL ${JAVA_LIBRARIES} ${JNI_LIBRARIES} ${Boost_LIBRARIES} ${LIBS})

# set the arch extension for the lib
IF(NOT APPLE)
  IF(${MACH_ARCH} MATCHES "64")
	#64bit
	message("set 64bit extension") 
	SET(NEW_SUFFIX "64")
  ELSE()
	# 32bit
	message("set 32bit extension") 
	SET(NEW_SUFFIX "32")
  ENDIF()
  IF(WIN32)
	SET(NEW_SUFFIX ${NEW_SUFFIX}.dll)	
  ELSE()
	# unix linux
	SET(NEW_SUFFIX ${NEW_SUFFIX}.so)	
  ENDIF()

  SET_TARGET_PROPERTIES(SimpleKDL PROPERTIES SUFFIX ${NEW_SUFFIX})
  

ENDIF()

# only for linux
IF(UNIX)
  IF(${MACH_ARCH} MATCHES "32")
	SET_TARGET_PROPERTIES(SimpleKDL PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")
  ENDIF()
ENDIF()

set(P5_JAR "${P5_JAR}")
set(P5_JAR_OPENGL ${P5_JAR_OPENGL})

# get full paths, otherwise javac/javadoc has problems with multiple jar locations
GET_FILENAME_COMPONENT(P5_JAR "${P5_JAR}" ABSOLUTE)
GET_FILENAME_COMPONENT(P5_JAR_OPENGL "${P5_JAR_OPENGL}" ABSOLUTE)
GET_FILENAME_COMPONENT(P5_JAR_JOGL "${P5_JAR_JOGL}" ABSOLUTE)

ADD_CUSTOM_COMMAND(TARGET SimpleKDL
				   POST_BUILD
				   COMMAND cmake -E echo "-----------------------------"
				   COMMAND cmake -E echo "Compiling Java files..."
				   COMMAND cmake -E make_directory ./SimpleKDL
				   COMMAND cmake -E make_directory ${CMAKE_SWIG_OUTDIR}
				   #COMMAND cmake -E copy_directory ${P5_WRAPPER} ${CMAKE_SWIG_OUTDIR}
				   # cmake copy doesn't support wildcards, otherwise it just copies the hidden folder of svn, etc.
				   # doesn't works on windows backslash
				   COMMAND cp "${P5_WRAPPER}*.java" "${CMAKE_SWIG_OUTDIR}"
				   #COMMAND cmake -E copy ${P5_WRAPPER_FILE} ${CMAKE_SWIG_OUTDIR}
                                   COMMAND ${JAVA_COMPILE2} -classpath \"${P5_JAR}:${P5_JAR_OPENGL}:${P5_JAR_JOGL}\" ${CMAKE_SWIG_OUTDIR}/*.java -d ./
				   COMMAND cmake -E echo "-----------------------------"
				   COMMAND cmake -E echo "Creating jar file..."
				   COMMAND ${JAVA_ARCHIVE2} cvf SimpleKDL.jar SimpleKDL
                                   COMMAND cmake -E echo "-----------------------------"
				   COMMAND cmake -E echo "Creating doc files..."
				   COMMAND cmake -E make_directory ./doc
				   #COMMAND ${JAVA_DOC} -quiet -author -public -nodeprecated -nohelp -d ./doc  -version ${P5_WRAPPER}/*.java -version ${CMAKE_SWIG_OUTDIR}/ContextWrapper.java)
                                   COMMAND ${JAVA_DOC} -classpath \"${P5_JAR}:${P5_JAR_OPENGL}:${P5_JAR_JOGL}\" -quiet -author -public -nodeprecated -nohelp -d ./doc  -version ${CMAKE_SWIG_OUTDIR}/*.java)
