local runtime = {}
runtime.name = "pbr-runtime"
runtime.path = "./runtime"
runtime.kind = "ConsoleApp"
runtime.language = "C++"
runtime.cppdialect = "C++Latest"

runtime.files = function()
  files {
    "./src/**.cpp" ,
    "./src/**.hpp" ,
  }
end

runtime.include_dirs = function()
  includedirs { "./src" }
end

runtime.windows_configuration = function()
  systemversion "latest"
  entrypoint "WinMainCRTStartup"
end

runtime.components = {}
runtime.components["pbr-renderer"] = { "%{wks.location}/renderer/src" }

AddProject(runtime)
