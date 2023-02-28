add_library(scanmatching SHARED)
target_sources(scanmatching PRIVATE scanmatching.cpp)
target_include_directories(scanmatching PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(scanmatching urg serialport speak)

add_library(urg SHARED IMPORTED GLOBAL)
set_target_properties(urg
    PROPERTIES
        IMPORTED_LOCATION ${PROJECT_BINARY_DIR}/out/liburg.dll
        IMPORTED_IMPLIB ${PROJECT_BINARY_DIR}/out/liburg.dll.a
        INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/include)

add_library(serialport SHARED IMPORTED GLOBAL)
set_target_properties(serialport
    PROPERTIES
        IMPORTED_LOCATION ${PROJECT_BINARY_DIR}/out/libserialport.dll
        IMPORTED_IMPLIB ${PROJECT_BINARY_DIR}/out/libserialport.dll.a
        INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/include)

add_library(speak SHARED IMPORTED GLOBAL)
set_target_properties(speak
    PROPERTIES
        IMPORTED_LOCATION ${PROJECT_BINARY_DIR}/out/libspeak.dll
        IMPORTED_IMPLIB ${PROJECT_BINARY_DIR}/out/libspeak.dll.a
        INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/include)

add_library(timer SHARED IMPORTED GLOBAL)
set_target_properties(timer
    PROPERTIES
        IMPORTED_LOCATION ${PROJECT_BINARY_DIR}/out/libtimer.dll
        IMPORTED_IMPLIB ${PROJECT_BINARY_DIR}/out/libtimer.dll.a
        INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/include)