cmake_minimum_required(VERSION 3.8)
project(atlas_vehicle_interface)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ackermann_model_contract
    test/test_ackermann_model_contract.py
  )
endif()

ament_package()
