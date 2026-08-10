if(NOT TARGET vendor_robot_sdk_vendor::cr_sdk)
  add_library(vendor_robot_sdk_vendor::cr_sdk SHARED IMPORTED)
  set_target_properties(vendor_robot_sdk_vendor::cr_sdk PROPERTIES
    IMPORTED_LOCATION "${vendor_robot_sdk_vendor_DIR}/../../../lib/libcr_sdk.so"
    INTERFACE_INCLUDE_DIRECTORIES "${vendor_robot_sdk_vendor_DIR}/../../../include/vendor_robot_sdk_vendor")
endif()
