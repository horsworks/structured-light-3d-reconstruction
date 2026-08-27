add_test(
  NAME environment_smoke_test
  COMMAND sl3d_info
)

add_executable(
  test_fringe
  test_fringe.cpp
)

target_link_libraries(
  test_fringe
  PRIVATE
    sl3d
)

add_test(
  NAME fringe_generation_test
  COMMAND test_fringe
)

add_executable(
  test_phase
  test_phase.cpp
)

target_link_libraries(
  test_phase
  PRIVATE
    sl3d
)

add_test(
  NAME wrapped_phase_test
  COMMAND test_phase
)

add_executable(
  test_phase_unwrap
  test_phase_unwrap.cpp
)

target_link_libraries(
  test_phase_unwrap
  PRIVATE
    sl3d
)

add_test(
  NAME phase_unwrap_test
  COMMAND test_phase_unwrap
)