# ==============================================
# 	Alumno  :   jhon charaja
# 	Info	: 	this file searches for Pinocchio 
# 				includes and library files.
# ==============================================

# prints information message
MESSAGE(STATUS "===========================")
MESSAGE(STATUS " Finding Pinocchio ...")
MESSAGE(STATUS "===========================")


# initializes information variables
SET (PIN_FOUND FALSE)

# Pinocchio directory path
set (PIN_INCLUDE_DIR $ENV{pin_include_dir} )

# Pinocchio library path
FIND_LIBRARY (PIN_LIBRARY NAMES pinocchio
	PATHS
	$ENV{pin_lib}
)	


# information about the success of finding the Pinocchio libraries
IF (NOT PIN_LIBRARY)
	MESSAGE (ERROR "Could not find Pinocchio library")
ENDIF (NOT PIN_LIBRARY)

IF (PIN_INCLUDE_DIR AND PIN_LIBRARY)
	SET (PIN_FOUND TRUE)
	MESSAGE(STATUS "Found Pinocchio: ${PIN_LIBRARY}")
ENDIF (PIN_INCLUDE_DIR AND PIN_LIBRARY)	


MARK_AS_ADVANCED (
	PIN_INCLUDE_DIR
	PIN_LIBRARY
	)

