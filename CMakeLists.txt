cmake_minimum_required(VERSION 3.20)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(OpenVR-SpaceOverride)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The Vulkan validation layer adds extra reporting that also surfaces validation    
# issues originating from external sources (e.g. SteamVR). Defaults on.
option(ENABLE_VULKAN_VALIDATION "Enable Vulkan validation layers" OFF)

set(OpenVR_ROOT ${CMAKE_SOURCE_DIR}/3rdparty/OpenVR)

find_package(OpenVR REQUIRED)

# Include rest of the CMake bindings
add_subdirectory(3rdparty)

message(STATUS "ENABLE_VULKAN_VALIDATION = ${ENABLE_VULKAN_VALIDATION}")

# The UI fonts are compiled in as byte arrays named Font_<file name>, so nothing is loaded at runtime.
add_library(EmbeddedFonts STATIC)
foreach(font
    geist/Geist-Regular.ttf
    geist/Geist-Medium.ttf
    geist/Geist-SemiBold.ttf
    geist/GeistMono-Regular.ttf
    phosphor/Phosphor.ttf
)
    get_filename_component(name "${font}" NAME_WE)
    string(MAKE_C_IDENTIFIER "Font_${name}" symbol)
    set(output "${CMAKE_CURRENT_BINARY_DIR}/fonts/${symbol}.cpp")
    add_custom_command(
        OUTPUT "${output}"
        COMMAND "${CMAKE_COMMAND}" "-DINPUT=${CMAKE_SOURCE_DIR}/resources/fonts/${font}" "-DOUTPUT=${output}" "-DSYMBOL=${symbol}"
            -P "${CMAKE_SOURCE_DIR}/cmake/EmbedFile.cmake"
        DEPENDS "${CMAKE_SOURCE_DIR}/resources/fonts/${font}" "${CMAKE_SOURCE_DIR}/cmake/EmbedFile.cmake"
        VERBATIM
    )
    target_sources(EmbeddedFonts PRIVATE "${output}")
endforeach()

add_executable(OpenVR-SpaceOverride
    "src/overlay/Main.cpp"
    "src/overlay/VulkanRenderer.cpp"
    "src/overlay/ImGuiWindow.cpp"
    "src/overlay/Calibration.cpp"
    "src/overlay/Configuration.cpp"
    "src/overlay/IPCClient.cpp"
    "src/overlay/UserInterface.cpp"
    "src/overlay/SystemFonts.cpp"
    "src/overlay/VRState.cpp"
)

target_compile_features(OpenVR-SpaceOverride PUBLIC cxx_std_20)

target_include_directories(OpenVR-SpaceOverride
    PRIVATE
        "${CMAKE_SOURCE_DIR}/include/overlay"
        "${CMAKE_SOURCE_DIR}/include/shared"
)

target_compile_definitions(OpenVR-SpaceOverride
    PRIVATE
        _HAS_DEPRECATED_NEGATORS=1
        _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
)

if (ENABLE_VULKAN_VALIDATION)
    target_compile_definitions(OpenVR-SpaceOverride PRIVATE ENABLE_VULKAN_VALIDATION)
endif()

if (MSVC)
    target_compile_options(OpenVR-SpaceOverride PRIVATE /bigobj)
endif()

# Windows executable icon
if (WIN32)
    target_sources(OpenVR-SpaceOverride PRIVATE "resources/winicon.rc")
endif()


target_link_libraries(OpenVR-SpaceOverride
    PRIVATE
        OpenVR::API
        ImGui
        Eigen
        PicoJSON
        EmbeddedFonts
)

add_library(driver_spaceoverride SHARED
    "src/driver/Main.cpp"
    "src/driver/ServerTrackedDeviceProvider.cpp"
    "src/driver/IPCServer.cpp"
    "src/driver/InterfaceHookInjector.cpp"
    "src/driver/Hooking.cpp"
    "src/driver/Logging.cpp"
    "src/driver/KalmanFilter.cpp"
)

target_compile_features(driver_spaceoverride PRIVATE cxx_std_20)

target_include_directories(driver_spaceoverride PRIVATE
    "${CMAKE_SOURCE_DIR}/include/driver"
    "${CMAKE_SOURCE_DIR}/include/shared"
)

target_link_libraries(driver_spaceoverride PRIVATE
    OpenVR::Driver
    MinHook
)

set_property(TARGET driver_spaceoverride PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
set_target_properties(driver_spaceoverride PROPERTIES PREFIX "")
set(SPACE_OVERRIDE_LOGIC_SOURCES
	"src/overlay/AcquisitionMath.cpp"
	"src/overlay/SamplerPolicy.cpp"
	"src/overlay/CalibrationMath.cpp"
	"src/overlay/AttemptLifecycle.cpp"
	"src/overlay/DesiredState.cpp"
	"src/overlay/ProfileCodec.cpp"
	"src/overlay/AutoAcquisition.cpp"
	"src/overlay/ManualDetection.cpp"
	"src/overlay/OverlayLog.cpp"
	"src/overlay/SpaceMemory.cpp"
)
target_sources(OpenVR-SpaceOverride PRIVATE ${SPACE_OVERRIDE_LOGIC_SOURCES})

add_executable(SpaceOverrideTests EXCLUDE_FROM_ALL
	"tests/SpaceOverrideTests.cpp"
	${SPACE_OVERRIDE_LOGIC_SOURCES}
)
target_include_directories(SpaceOverrideTests PRIVATE
	"${CMAKE_SOURCE_DIR}/include/overlay"
	"${CMAKE_SOURCE_DIR}/include/shared"
	"${OpenVR_INCLUDE_DIR}"
)
target_link_libraries(SpaceOverrideTests PRIVATE Eigen PicoJSON)
target_compile_definitions(SpaceOverrideTests PRIVATE
	_HAS_DEPRECATED_NEGATORS=1
	_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
)
if (MSVC)
	target_compile_options(SpaceOverrideTests PRIVATE /bigobj)
endif()
enable_testing()
add_test(NAME SpaceOverrideTests COMMAND SpaceOverrideTests)

# Renders the overlay UI to PNGs without SteamVR or a GPU (UiPreview <output folder> [scene...]),
# and UiPreview --check clicks through every control. It builds its own Dear ImGui with the test engine
# hooks, which is how the checks find items on screen.
add_executable(UiPreview EXCLUDE_FROM_ALL
	"tools/UiPreview/UiPreview.cpp"
	"tools/UiPreview/Checks.cpp"
	"tools/UiPreview/PreviewState.cpp"
	"tools/UiPreview/SoftwareRenderer.cpp"
	"src/overlay/UserInterface.cpp"
	"src/overlay/SystemFonts.cpp"
	"${CMAKE_SOURCE_DIR}/3rdparty/ImGui/imgui.cpp"
	"${CMAKE_SOURCE_DIR}/3rdparty/ImGui/imgui_draw.cpp"
	"${CMAKE_SOURCE_DIR}/3rdparty/ImGui/imgui_tables.cpp"
	"${CMAKE_SOURCE_DIR}/3rdparty/ImGui/imgui_widgets.cpp"
)
target_include_directories(UiPreview PRIVATE
	"${CMAKE_SOURCE_DIR}/include/overlay"
	"${CMAKE_SOURCE_DIR}/include/shared"
	"${CMAKE_SOURCE_DIR}/3rdparty/ImGui"
	"${OpenVR_INCLUDE_DIR}"
)
target_link_libraries(UiPreview PRIVATE SDL3::SDL3 Eigen PicoJSON EmbeddedFonts)
target_compile_definitions(UiPreview PRIVATE
	IMGUI_ENABLE_TEST_ENGINE
	_HAS_DEPRECATED_NEGATORS=1
	_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
)
if (MSVC)
	target_compile_options(UiPreview PRIVATE /bigobj)
endif()
