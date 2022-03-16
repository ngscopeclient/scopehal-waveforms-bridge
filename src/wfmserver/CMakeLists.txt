###############################################################################
#C++ compilation
add_executable(wfmserver
	DigilentSCPIServer.cpp
	WaveformServerThread.cpp
	main.cpp
)

###############################################################################
#Linker settings
target_link_libraries(wfmserver
	xptools
	log
	dwf
	scpi-server-tools
	)

