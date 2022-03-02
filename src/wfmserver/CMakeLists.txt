###############################################################################
#C++ compilation
add_executable(wfmserver
	ScpiServerThread.cpp
	WaveformServerThread.cpp
	main.cpp
)

###############################################################################
#Linker settings
target_link_libraries(wfmserver
	xptools
	log
	dwf
	)

