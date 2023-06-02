dofile "gkit.lua"

 -- description des projets		 
projects = {
	"tp1"--,
	-- "tp2",
}

for i, name in ipairs(projects) do
    project(name)
        language "C++"
        kind "ConsoleApp"
        targetdir "bin"
        files ( gkit_files )
        files { "projets/" .. name .. ".cpp" }
    project("bvh")
        language "C++"
        kind "ConsoleApp"
        targetdir "bin"
        buildoptions ( "-std=c++17" )
        files ( gkit_files )
        files { "projets/bvh-etu/src/*.cpp" }

    project("tp2")
        language "C++"
        kind "ConsoleApp"
        targetdir "bin"
        buildoptions ( "-std=c++17" )
        files ( gkit_files )
        files { "projets/tp2/src/*.cpp" }

end
