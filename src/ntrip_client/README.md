NTRIP Client for ROS 2. Corrections are published as a multi-byte array on the **/ntrip_client/rtcm3_0** topic to abstract this functionality from the end consumer application.

The launch file expects a configuration file at /var/lib/ntrip-config/ called *service.yaml* with parameters required to connect to your caster.
