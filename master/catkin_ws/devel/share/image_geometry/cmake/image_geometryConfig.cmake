# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(image_geometry_CONFIG_INCLUDED)
  return()
endif()
set(image_geometry_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
set(image_geometry_SOURCE_PREFIX /home/test/catkin_ws/src/vision_opencv/image_geometry)
set(image_geometry_DEVEL_PREFIX /home/test/catkin_ws/devel)
set(image_geometry_INSTALL_PREFIX )
set(image_geometry_PREFIX ${image_geometry_DEVEL_PREFIX})

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'image_geometry' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(image_geometry_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/test/catkin_ws/src/vision_opencv/image_geometry/include;/usr/include/opencv4 " STREQUAL " ")
  set(image_geometry_INCLUDE_DIRS "")
  set(_include_dirs "/home/test/catkin_ws/src/vision_opencv/image_geometry/include;/usr/include/opencv4")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT "http://www.ros.org/wiki/image_geometry " STREQUAL " ")
    set(_report "Check the website 'http://www.ros.org/wiki/image_geometry' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Vincent Rabaud <vincent.rabaud@gmail.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    # don't try to guess include dir if we installed into the default
    elseif("" STREQUAL "/usr")
      continue()
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${image_geometry_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'image_geometry' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'image_geometry' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/test/catkin_ws/src/vision_opencv/image_geometry/${idir}'.  ${_report}")
    endif()
    if(NOT "${include}" STREQUAL "/usr/include")
    _list_append_unique(image_geometry_INCLUDE_DIRS ${include})
    endif()
  endforeach()
endif()

set(libraries "image_geometry;-lopencv_calib3d;-lopencv_core;-lopencv_dnn;-lopencv_features2d;-lopencv_flann;-lopencv_highgui;-lopencv_imgcodecs;-lopencv_imgproc;-lopencv_ml;-lopencv_objdetect;-lopencv_photo;-lopencv_stitching;-lopencv_video;-lopencv_videoio;-lopencv_alphamat;-lopencv_aruco;-lopencv_barcode;-lopencv_bgsegm;-lopencv_bioinspired;-lopencv_ccalib;-lopencv_cvv;-lopencv_datasets;-lopencv_dnn_objdetect;-lopencv_dnn_superres;-lopencv_dpm;-lopencv_face;-lopencv_freetype;-lopencv_fuzzy;-lopencv_hdf;-lopencv_hfs;-lopencv_img_hash;-lopencv_intensity_transform;-lopencv_line_descriptor;-lopencv_mcc;-lopencv_optflow;-lopencv_phase_unwrapping;-lopencv_plot;-lopencv_quality;-lopencv_rapid;-lopencv_reg;-lopencv_rgbd;-lopencv_saliency;-lopencv_shape;-lopencv_stereo;-lopencv_structured_light;-lopencv_superres;-lopencv_surface_matching;-lopencv_text;-lopencv_tracking;-lopencv_videostab;-lopencv_viz;-lopencv_wechat_qrcode;-lopencv_ximgproc;-lopencv_xobjdetect;-lopencv_xphoto")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND image_geometry_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND image_geometry_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT image_geometry_NUM_DUMMY_TARGETS)
      set(image_geometry_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::image_geometry::wrapped-linker-option${image_geometry_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR image_geometry_NUM_DUMMY_TARGETS "${image_geometry_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::image_geometry::wrapped-linker-option${image_geometry_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND image_geometry_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND image_geometry_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND image_geometry_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/test/catkin_ws/devel/lib;/home/test/catkin_ws/devel/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(image_geometry_LIBRARY_DIRS ${lib_path})
      list(APPEND image_geometry_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'image_geometry'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND image_geometry_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(image_geometry_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${image_geometry_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "sensor_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 image_geometry_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${image_geometry_dep}_FOUND)
      find_package(${image_geometry_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${image_geometry_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(image_geometry_INCLUDE_DIRS ${${image_geometry_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(image_geometry_LIBRARIES ${image_geometry_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${image_geometry_dep}_LIBRARIES})
  _list_append_deduplicate(image_geometry_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(image_geometry_LIBRARIES ${image_geometry_LIBRARIES})

  _list_append_unique(image_geometry_LIBRARY_DIRS ${${image_geometry_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(image_geometry_EXPORTED_TARGETS ${${image_geometry_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${image_geometry_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
