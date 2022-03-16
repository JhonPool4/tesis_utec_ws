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
set (EIGEN_INCLUDE_DIR $ENV{eigen_include_dir})

MARK_AS_ADVANCED (
	EIGEN_INCLUDE_DIR
	)

