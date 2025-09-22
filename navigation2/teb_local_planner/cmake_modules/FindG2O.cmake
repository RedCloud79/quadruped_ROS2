# Locate the g2o libraries
# A general framework for graph optimization.
#
# This module defines
# G2O_FOUND, if false, do not try to link against g2o
# G2O_LIBRARIES, path to the libg2o
# G2O_INCLUDE_DIR, where to find the g2o header files
#
# Niko Suenderhauf <niko@etit.tu-chemnitz.de>
# Adapted by Felix Endres <endres@informatik.uni-freiburg.de>
# Patched to be more robust on modern systems

IF(UNIX)

  MESSAGE(STATUS "Searching for g2o ...")

  # Look for a key header to verify include path
  FIND_PATH(G2O_INCLUDE_DIR
    NAMES base_vertex.h
    PATHS /usr/local/include/g2o /usr/include/g2o
    PATH_SUFFIXES core)

  IF (G2O_INCLUDE_DIR)
    MESSAGE(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
  ENDIF ()

  # === Required libraries ===
  FIND_LIBRARY(G2O_CORE_LIB NAMES g2o_core PATHS /usr/local/lib /usr/lib)
  FIND_LIBRARY(G2O_STUFF_LIB NAMES g2o_stuff PATHS /usr/local/lib /usr/lib)
  FIND_LIBRARY(G2O_TYPES_SLAM2D_LIB NAMES g2o_types_slam2d PATHS /usr/local/lib /usr/lib)
  FIND_LIBRARY(G2O_TYPES_SLAM3D_LIB NAMES g2o_types_slam3d PATHS /usr/local/lib /usr/lib)
  FIND_LIBRARY(G2O_SOLVER_CSPARSE_LIB NAMES g2o_solver_csparse PATHS /usr/local/lib /usr/lib)

  # === Optional libraries ===
  FIND_LIBRARY(G2O_SOLVER_CHOLMOD_LIB NAMES g2o_solver_cholmod PATHS /usr/local/lib /usr/lib)
  FIND_LIBRARY(G2O_SOLVER_PCG_LIB NAMES g2o_solver_pcg PATHS /usr/local/lib /usr/lib)
  FIND_LIBRARY(G2O_CSPARSE_EXTENSION_LIB NAMES g2o_csparse_extension PATHS /usr/local/lib /usr/lib)
  FIND_LIBRARY(G2O_INCREMENTAL_LIB NAMES g2o_incremental PATHS /usr/local/lib /usr/lib)

  # Collect all found libraries
  set(G2O_LIBRARIES "")
  foreach(lib 
      ${G2O_CORE_LIB} ${G2O_STUFF_LIB}
      ${G2O_TYPES_SLAM2D_LIB} ${G2O_TYPES_SLAM3D_LIB}
      ${G2O_SOLVER_CSPARSE_LIB}
      ${G2O_SOLVER_CHOLMOD_LIB} ${G2O_SOLVER_PCG_LIB}
      ${G2O_CSPARSE_EXTENSION_LIB} ${G2O_INCREMENTAL_LIB})
    if(lib AND NOT lib MATCHES "NOTFOUND")
      list(APPEND G2O_LIBRARIES ${lib})
    endif()
  endforeach()

  # Check essential components
  if(G2O_INCLUDE_DIR AND G2O_CORE_LIB AND G2O_STUFF_LIB)
    set(G2O_FOUND "YES")
    message(STATUS "Found libg2o: ${G2O_LIBRARIES}")
  else()
    message(FATAL_ERROR "Could not find essential g2o components (core/stuff)!")
  endif()

ENDIF(UNIX)
