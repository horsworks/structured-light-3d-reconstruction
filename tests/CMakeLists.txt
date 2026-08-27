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