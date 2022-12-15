#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <fdeep/fdeep.hpp>

extern "C" __EXPORT int nn_example_main(int argc, char *argv[]);
int nn_example_main(int argc, char *argv[])
{
	PX4_INFO("Hello, I am a dynamically loaded module.");

	const auto model = fdeep::load_model("fdeep_model.json");
    const auto result = model.predict(
        {fdeep::tensor(fdeep::tensor_shape(static_cast<std::size_t>(4)),
        std::vector<float>{1, 2, 3, 4})});
    std::cout << fdeep::show_tensors(result) << std::endl;

	return 0;
}
