# Writes INPUT into OUTPUT as a C++ byte array named SYMBOL, with its length in SYMBOL_size.
# Run with: cmake -DINPUT=<file> -DOUTPUT=<file.cpp> -DSYMBOL=<name> -P EmbedFile.cmake

file(READ "${INPUT}" bytes HEX)
string(LENGTH "${bytes}" length)
math(EXPR size "${length} / 2")
# 16 bytes per line keeps the generated source readable for compilers and editors.
string(REGEX REPLACE "(................................)" "\\1\n" bytes "${bytes}")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," bytes "${bytes}")
get_filename_component(name "${INPUT}" NAME)
file(WRITE "${OUTPUT}"
	"// Generated from ${name} by cmake/EmbedFile.cmake.\n"
	"#include <cstddef>\n"
	"extern const unsigned char ${SYMBOL}[] = {\n${bytes}\n};\n"
	"extern const std::size_t ${SYMBOL}_size = ${size};\n")
