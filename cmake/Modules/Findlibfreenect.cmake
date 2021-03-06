# - Try to find libfreenect

if (LIBFREENECT_LIBRARIES AND LIBFREENCT_INCLUDE_DIRS)
  # in cache already
  set(LIBFREENECT_FOUND TRUE)
else (LIBFREENECT_LIBRARIES AND LIBFREENCT_INCLUDE_DIRS)
  find_path(LIBFREENECT_INCLUDE_DIR
    NAMES
	libfreenect/libfreenect.h
	libfreenect/libfreenect_sync.h
    PATHS
      /usr/include
      /usr/local/include
  )

  find_library(LIBFREENECT_LIBRARY
    NAMES
      freenect
      freenect_sync
      
    PATHS
      /usr/lib
      /usr/local/lib
  )

  set(LIBFREENECT_INCLUDE_DIRS
    ${LIBFREENECT_INCLUDE_DIR}
  )
  set(LIBFREENECT_LIBRARIES
    ${LIBFREENECT_LIBRARY}
)

  if (LIBFREENECT_INCLUDE_DIRS AND LIBFREENECT_LIBRARIES)
     set(LIBFREENECT_FOUND TRUE)
  endif (LIBFREENECT_INCLUDE_DIRS AND LIBFREENECT_LIBRARIES)

  if (LIBFREENECT_FOUND)
    if (NOT LIBFREENECT_FIND_QUIETLY)
      message(STATUS "Found libfreenect:")
	  message(STATUS " - Includes: ${LIBFREENECT_INCLUDE_DIRS}")
	  message(STATUS " - Libraries: ${LIBFREENECT_LIBRARIES}")
    endif (NOT LIBFREENECT_FIND_QUIETLY)
  else (LIBFREENECT_FOUND)
    if (LIBFREENECT_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find libfreenect")
    endif (LIBFREENECT_FIND_REQUIRED)
  endif (LIBFREENECT_FOUND)

  # show the LIBUSB_1_INCLUDE_DIRS and LIBUSB_1_LIBRARIES variables only in the advanced view
  mark_as_advanced(LIBFREENECT_INCLUDE_DIRS LIBFREENECT_LIBRARIES)

endif (LIBFREENECT_LIBRARIES AND LIBFREENCT_INCLUDE_DIRS)
