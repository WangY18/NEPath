find_package(Catch2 3 REQUIRED)

add_executable(test_iqop_geometry
    test_iqop_geometry.cpp
)

target_link_libraries(test_iqop_geometry
    PRIVATE
        NEPath::NEPath
        Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(test_iqop_geometry)

add_executable(test_iqop_subproblem
    test_iqop_subproblem.cpp
)

target_link_libraries(test_iqop_subproblem
    PRIVATE
        NEPath::NEPath
        Catch2::Catch2WithMain
)

catch_discover_tests(test_iqop_subproblem)

add_executable(test_iqop_ifopt
    test_iqop_ifopt.cpp
)

target_link_libraries(test_iqop_ifopt
    PRIVATE
        NEPath::NEPath
        Catch2::Catch2WithMain
)

catch_discover_tests(test_iqop_ifopt)

add_executable(test_iqop_scp
    test_iqop_scp.cpp
)

target_link_libraries(test_iqop_scp
    PRIVATE
        NEPath::NEPath
        Catch2::Catch2WithMain
)

catch_discover_tests(test_iqop_scp)

add_executable(test_iqop_identity
    test_iqop_identity.cpp
)

target_link_libraries(test_iqop_identity
    PRIVATE
        NEPath::NEPath
        Catch2::Catch2WithMain
)

catch_discover_tests(test_iqop_identity)

add_executable(iqop_ipopt_reference
    iqop_ipopt_reference.cpp
)

target_link_libraries(iqop_ipopt_reference
    PRIVATE
        NEPath::NEPath
)
