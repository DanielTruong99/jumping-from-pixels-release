#add_library(inih STATIC ini.c ini.h INIReader.h INIReader.cpp)
add_library(inih SHARED ini.c ini.h INIReader.h INIReader.cpp)
install(TARGETS inih DESTINATION lib)
