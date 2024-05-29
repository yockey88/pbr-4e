require("ymake")

local configuration = {}
configuration.wks_name = "PBR"
configuration.architecture = "x64"
configuration.start_project = "runtime"
configuration.cpp_dialect = "C++latest"
configuration.static_runtime = "On"
configuration.target_dir = "%{wks.location}/bin/%{cfg.buildcfg}/%{prj.name}"
configuration.obj_dir = "%{wks.location}/bin_obj/%{cfg.buildcfg}/%{prj.name}"

-- TODO: upgrade platform/configuration system
configuration.build_configurations = { "Debug", "Release" }
configuration.platforms = { "Windows" }

configuration.groups = {
  ["renderer"] = { "./renderer" } ,
  ["runtime"] = { "./runtime" } ,
  ["testing"] = { "./tests" } ,
}

local glad = {}
glad.name = "glad"
glad.path = "./external/glad"
glad.include_dir = "%{wks.location}/external/glad/include"
glad.lib_name = "glad"
glad.lib_dir = "%{wks.location}/bin/Debug/glad"

local glm = {}
glm.name = "glm"
glm.include_dir = "%{wks.location}/external/glm"

local imgui = {}
imgui.name = "imgui"
imgui.path = "./external/imgui"
imgui.include_dir = "%{wks.location}/external/imgui"
imgui.lib_name = "imgui"
imgui.lib_dir = "%{wks.location}/bin/Debug/imgui"

local sdl2 = {}
sdl2.name = "sdl2"
sdl2.include_dir = "%{wks.location}/external/SDL2/SDL2"
sdl2.lib_dir = "%{wks.location}/external/SDL2/lib/%{cfg.buildcfg}"
sdl2.lib_name = "SDL2"
sdl2.debug_lib_name = "SDL2d"
sdl2.configurations = { "Debug", "Release" }

local spdlog = {}
spdlog.name = "spdlog"
spdlog.path = "./external/spdlog"
spdlog.include_dir = "%{wks.location}/external/spdlog/include"
spdlog.lib_name = "spdlog"
spdlog.lib_dir = "%{wks.location}/bin/Debug/spdlog"

local stb = {}
stb.name = "stb"
stb.include_dir = "%{wks.location}/external/stb"

AddDependency(glad)
AddDependency(glm)
AddDependency(imgui)
AddDependency(sdl2)
AddDependency(spdlog)
AddDependency(stb)

CppWorkspace(configuration)
