set(SOURCES
    "src/trajectory_from_roadmatrix.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/trajectory_from_roadmatrix.h"
)

include_directories(include)
add_library(trajectory_from_roadmatrix MODULE ${SOURCES} ${HEADERS})
target_link_libraries(trajectory_from_roadmatrix PRIVATE lmscore street_environment)
