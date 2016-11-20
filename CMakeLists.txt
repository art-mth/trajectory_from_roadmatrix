set(SOURCES
    "src/trajectory_from_roadmatrix.cpp"
    "src/trajectory_from_roadmatrix_impl.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/trajectory_from_roadmatrix.h"
    "include/trajectory_from_roadmatrix_impl.h"
)

include_directories(include)
add_library(trajectory_from_roadmatrix SHARED ${SOURCES} ${HEADERS})
target_link_libraries(trajectory_from_roadmatrix PRIVATE lmscore street_environment)

if(LOCAL_GMOCK)
set(TESTS
    "src/trajectory_from_roadmatrix_impl_test.cpp"
)
add_executable(trajectory_from_roadmatrix_impl_test ${TESTS})
target_link_libraries(trajectory_from_roadmatrix_impl_test PRIVATE
    gtest gmock gmock_main street_environment lms_math trajectory_from_roadmatrix)
endif(LOCAL_GMOCK)
