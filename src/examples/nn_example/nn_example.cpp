#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <fdeep/fdeep.hpp>

extern "C" __EXPORT int nn_example_main(int argc, char *argv[]);
int nn_example_main(int argc, char *argv[])
{
	PX4_INFO("Hello, I am a dynamically loaded module.");

	PX4_INFO("Argv:");

	for (int i = 0; i < argc; ++i) {
		PX4_INFO("  %d: %s", i, argv[i]);
	}

	return 0;
}
