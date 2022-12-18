#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/posix.h>
#include <fdeep/fdeep.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>

extern "C" __EXPORT int nn_example_main(int argc, char *argv[]);

int nn_example_main(int argc, char *argv[])
{
    // subscribe to sensor_combined topic
	int actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	// limit the update rate to 5 Hz
	orb_set_interval(actuator_controls_sub, 5);

    // subscribe to sensor_combined topic
	int angular_acc_sub = orb_subscribe(ORB_ID(vehicle_angular_acceleration));

    // subscribe to sensor_combined topic
	int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    // publish to output topics
    struct actuator_outputs_s cmd;
	memset(&cmd, 0, sizeof(cmd));
	orb_advert_t cmd_pub = orb_advertise(ORB_ID(actuator_outputs), &cmd);

    /* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = actuator_controls_sub,   .events = POLLIN },
        { .fd = angular_acc_sub,   .events = POLLIN },
        { .fd = local_position_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

    int error_counter = 0;
    // load h5 model
    const auto model = fdeep::load_model("fdeep_model.json");

    for(int i = 0; i < 100; i++){

        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 10);

        /* handle the poll result */
            if (poll_ret == 0) {
                /* this means none of our providers is giving us data */
                PX4_ERR("Got no data within a second");

            } else if (poll_ret < 0) {
                /* this is seriously bad - should be an emergency */
                if (error_counter < 10 || error_counter % 50 == 0) {
                    /* use a counter to prevent flooding (and slowing us down) */
                    PX4_ERR("ERROR return value from poll(): %d", poll_ret);
                }
                error_counter++;
            } 
        else {
            /* this means none of our providers is giving us data */
            PX4_INFO("Got data");

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct actuator_controls_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(actuator_controls_0), actuator_controls_sub, &raw);

                // mount message and predict
                const auto result = model.predict(
                    {fdeep::tensor(fdeep::tensor_shape(static_cast<std::size_t>(4)),
                    std::vector<float>{raw.control[0], raw.control[1], raw.control[2], raw.control[3]})});

                // std::string str = fdeep::show_tensors(result); 

                const auto values = result[0].to_vector();

                PX4_INFO("Result:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                                (double)values[0],
                                (double)values[1],
                                (double)values[2],
                                (double)values[3]);

                PX4_INFO("Errors:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                (double)(values[0] - raw.control[0],
                (double)values[1] - raw.control[1],
                (double)values[2] - raw.control[2],
                (double)values[3] - raw.control[3]);

                // publishes commands from nn to motors
                orb_publish(ORB_ID(actuator_controls_0), cmd_pub, &cmd);
            }
        }
    }
    PX4_INFO("Exiting");
	return 0;
}