# test/CMakeLists.txt

# テスト用の実行ファイルを作成
add_executable(test_sgp4_orbit
    test_sgp4_orbit.cpp
)

# インクルードディレクトリを設定
target_include_directories(test_sgp4_orbit
    PRIVATE
        ${CMAKE_SOURCE_DIR}/lib
        ${CMAKE_SOURCE_DIR}/include
        ${LIBSGP4_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIRS}
)

# SGP4orbit関連のソースを直接リンク
add_library(sgp4_orbit_test STATIC
    ${CMAKE_SOURCE_DIR}/lib/sgp4_orbit.cpp
)

target_include_directories(sgp4_orbit_test
    PRIVATE
        ${CMAKE_SOURCE_DIR}/lib
        ${CMAKE_SOURCE_DIR}/include
        ${LIBSGP4_INCLUDE_DIR}
)

# リンクするライブラリを設定
target_link_libraries(test_sgp4_orbit
    PRIVATE
        sgp4_orbit_test
        ${LIBSGP4_LIBRARY}
        gtest
        gtest_main
        pthread
)

# テストの実行をCTestに追加
add_test(NAME test_sgp4_orbit
         COMMAND test_sgp4_orbit)

# テストの作業ディレクトリを設定
set_tests_properties(test_sgp4_orbit
    PROPERTIES
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)