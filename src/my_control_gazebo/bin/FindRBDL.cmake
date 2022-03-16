# ==============================================
# 	Alumno  :   jhon charaja
# 	Info	: 	this file searches for RBDL includes 
# 				and library files.
# ==============================================

# prints information message
MESSAGE(STATUS "===========================")
MESSAGE(STATUS " Fingind RBDL and URDF ...")
MESSAGE(STATUS "===========================")

# initializes information variables
SET (RBDL_FOUND FALSE)
SET (RBDL_URDFReader_FOUND FALSE)

# RBDL directory path
FIND_PATH (RBDL_INCLUDE_DIR rbdl/rbdl.h
	HINTS
	$ENV{rbdl_include_dir}
	)
# RBDL library path
FIND_LIBRARY (RBDL_LIBRARY NAMES rbdl
	PATHS
	$ENV{rbdl_lib}
	)

# URDF Reader directory path 
FIND_PATH (RBDL_URDFReader_INCLUDE_DIR rbdl/addons/urdfreader/urdfreader.h
	HINTS
	$ENV{rbdl_urdfreader_include_dir}
	)
# URDF Reader library path
FIND_LIBRARY (RBDL_URDFReader_LIBRARY NAMES rbdl_urdfreader
	PATHS
	$ENV{rbdl_urdfreader_lib}			
	)

# information about the success of finding the RBDL libraries
IF (NOT RBDL_LIBRARY)
	MESSAGE (ERROR "Could not find RBDL library")
ENDIF (NOT RBDL_LIBRARY)

IF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)
	SET (RBDL_FOUND TRUE)
	MESSAGE(STATUS "Found RBDL: ${RBDL_LIBRARY}")
ENDIF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)


# information about the success of finding the URDFReader libraries
IF (NOT RBDL_URDFReader_LIBRARY)
	MESSAGE (ERROR "Could not find URDFReader library")
ENDIF (NOT RBDL_URDFReader_LIBRARY)

IF (RBDL_URDFReader_INCLUDE_DIR AND RBDL_URDFReader_LIBRARY)
	SET (RBDL_URDFReader_FOUND TRUE)
	MESSAGE(STATUS "FOUND URDF_READER: ${RBDL_URDFReader_LIBRARY}")
ENDIF (RBDL_URDFReader_INCLUDE_DIR AND RBDL_URDFReader_LIBRARY)


MARK_AS_ADVANCED (
	RBDL_INCLUDE_DIR
	RBDL_LIBRARY
	RBDL_URDFReader_INCLUDE_DIR
	RBDL_URDFReader_LIBRARY
	)

