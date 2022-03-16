# ==============================================
# 	Alumno  :   jhon charaja
# 	Info	: 	this file searches for Eigen 
# 				includes
# ==============================================

# prints information message
MESSAGE(STATUS "===========================")
MESSAGE(STATUS " Finding Eigen ...")
MESSAGE(STATUS "===========================")


# EIGEN directory path
set (EIGEN_INCLUDE_DIR "/home/jhon/Downloads/eigen-3.4.0")

MARK_AS_ADVANCED (
	EIGEN_INCLUDE_DIR
	)

