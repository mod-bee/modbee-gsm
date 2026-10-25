# Plain C++ examples, built on a host. The sketches live in examples/arduino
# and are built by the Arduino toolchain instead.

add_executable(socket_on_a_host socket_on_a_host.cpp)
target_link_libraries(socket_on_a_host PRIVATE modbee-gsm)
if(MSVC)
    target_compile_options(socket_on_a_host PRIVATE /W4)
else()
    target_compile_options(socket_on_a_host PRIVATE -Wall -Wextra)
endif()
