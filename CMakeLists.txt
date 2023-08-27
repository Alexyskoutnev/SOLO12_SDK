CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

project(
	SOLO12_SDK
	VERSION 0.1.0 
	LANGUAGES CXX
)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	option(DEBUG_BUILD "Build everything in debug mode" ON)
	option(BUILD_TESTS "Build tests" OFF) # we have no tests yet
	option(BUILD_EXAMPLES "Build examples" ON)
	
	option(USE_SINGLE_PRECISION "Use single precision floats" OFF)
	option(DRY_BUILD "Build without building masterboard sdk for debug" OFF)

	if(WIN32) # force dry build because masterboard sdk cannot be compiled on w*ndows
		message("You are on w*ndows, forcing dry build")
		set(DRY_BUILD ON)
		set(BUILD_EXAMPLES OFF)
	endif()

	message("Build Examples -> ${BUILD_EXAMPLES}")

	#* where to look for the project header and source files
	set(INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
	set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
	set(TEST_DIR ${CMAKE_CURRENT_LIST_DIR}/test)
	set(EXAMPLE_DIR ${CMAKE_CURRENT_LIST_DIR}/examples)

	#* tests, examples, benchmarks to compile
	set(TEST_NAMES
	)
	set(EXAMPLE_NAMES
		example
		example_pd
		example_imu_data_collection
	)

	#* source files
	file(GLOB_RECURSE MASTER_BOARD_SRC CONFIGURE_DEPENDS "${SRC_DIR}/master_board_sdk/*.cpp")
	file(GLOB_RECURSE COMMANDER_SRC CONFIGURE_DEPENDS "${SRC_DIR}/commander/*.cpp")

	#* set up output directories
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
	file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/data)

	#* project compile options
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)

	add_compile_options(
		-fdiagnostics-color=always #* colored debug output
		-fmessage-length=0 #* disable line wrapping (default=72 char) 
		-Wall #* all warnings
		-Wextra #* more warnings
		-Wpedantic #* more warnings
	)

	if(DEBUG_BUILD)
		add_compile_options(
			-g #* debug symbols
			-O0 #* no optimization, -Og may work better with some debuggers
		)
	else()
		add_compile_options(
			-O3 #* release optimization
		)
	endif()

	if(USE_SINGLE_PRECISION)
		add_compile_options(-DUSE_SINGLE_PRECISION) #* use single precision floats
	endif()

	if(DRY_BUILD)
		add_compile_options(-DDRY_BUILD) #* use single precision floats
	endif()

	#*************#
	#* Libraries *#
	#*************#
	if(NOT DRY_BUILD)
		add_library(master_board SHARED
			${MASTER_BOARD_SRC}
		)
		target_link_libraries(master_board pthread)
		target_include_directories(
			master_board PUBLIC 
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		)
	endif()

	#***************#
	#* Executables *#
	#***************#g
	add_executable(
		main
		${COMMANDER_SRC}			
	)
	target_include_directories(
		main PRIVATE 
		${INCLUDE_DIR}
		${matrix_rw_SOURCE_DIR}/include
		${rt_timer_SOURCE_DIR}/include
		${cxxopts_SOURCE_DIR}/include
		
	)
#	target_link_libraries(
#		main
#		${CURSES_LIBRARIES}
#	)
	target_compile_options(
		main PRIVATE
	)
	if(NOT DRY_BUILD)
		target_link_libraries(
			main 
			master_board
		)
	endif()
	
	#***********#
	#* Testing *#
	#***********#
	if(BUILD_TESTS)
		enable_testing()

		foreach(ELEMENT ${TEST_NAMES})
			add_executable(
				${ELEMENT}
				${TEST_DIR}/${ELEMENT}.cpp						
			)
			target_include_directories(
				${ELEMENT} PRIVATE 
				${INCLUDE_DIR}
				${matrix_rw_SOURCE_DIR}/include
				${rt_timer_SOURCE_DIR}/include
			)
			target_compile_options(
				${ELEMENT} PRIVATE
			)
			add_test(
				NAME ${ELEMENT} 
				COMMAND ${ELEMENT}
				WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
			)
		endforeach(ELEMENT ${TEST_NAMES})
	endif()

	#************#
	#* Examples *#
	#************#
	if(BUILD_EXAMPLES)
		foreach(ELEMENT ${EXAMPLE_NAMES})
			add_executable(
				${ELEMENT}
				${EXAMPLE_DIR}/${ELEMENT}.cpp						
			)
			target_include_directories(
				${ELEMENT} PRIVATE 
				${INCLUDE_DIR}
			)
			target_compile_options(
				${ELEMENT} PRIVATE
			)
			target_link_libraries(
				${ELEMENT} master_board
			)

		endforeach(ELEMENT ${EXAMPLE_NAMES})
	endif()

	#**************************#
	#* Executable permissions *#
	#**************************#
	#if(NOT DEBUG_BUILD)
		add_custom_command(
			TARGET main
			POST_BUILD
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			COMMAND scripts/set_permissions.sh
		)
	#endif()
endif()
