local renderer = {}
renderer.name = "pbr-renderer"
renderer.path = "./renderer"
renderer.kind = "StaticLib"
renderer.language = "C++"
renderer.cppdialect = "C++Latest"

renderer.files = function()
  files {
    "./src/**.cpp" ,
    "./src/**.hpp" ,
  }
end

renderer.include_dirs = function()
  includedirs { "./src" }
end

renderer.windows_configurations = function()
  systemversion "latest"
end

AddProject(renderer)
