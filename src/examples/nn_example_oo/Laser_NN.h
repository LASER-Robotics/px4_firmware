class Laser_NN final : public ModuleBase<Laser_NN>, public ModuleParams, public px4::ScheduledWorkItem{
    public:
        // subscribe to sensor_combined topic
        int actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
        // limit the update rate to 5 Hz
        orb_set_interval(actuator_controls_sub, 200);

        // subscribe to sensor_combined topic
        int angular_acc_sub = orb_subscribe(ORB_ID(vehicle_angular_acceleration));

        // subscribe to sensor_combined topic
        int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

        // publish to output topics
        struct actuator_outputs_s cmd;
        memset(&cmd, 0, sizeof(cmd));
        orb_advert_t cmd_pub = orb_advertise(ORB_ID(actuator_outputs), &cmd);

        Laser_NN::PrintExample();
}