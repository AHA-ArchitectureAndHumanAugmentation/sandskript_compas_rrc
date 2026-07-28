import time
import compas_rrc as rrc

PUMP_OUTPUT = "ABB_Scalable_IO_0_DO2"

ros = None

try:
    print("Connecting to ROS...")

    ros = rrc.RosClient()
    ros.run()

    abb = rrc.AbbClient(ros, "/rob1")

    print("Turning pump ON.")
    abb.send_and_wait(rrc.SetDigital(PUMP_OUTPUT, 1))

    time.sleep(5)

    print("Turning pump OFF.")
    abb.send_and_wait(rrc.SetDigital(PUMP_OUTPUT, 0))

    print("Pump IO test completed.")

except KeyboardInterrupt:
    print("\nTest stopped manually.")

except Exception as error:
    print("\nPump IO test failed:", type(error).__name__, error)

finally:
    if ros is not None:
        try:
            if ros.is_connected:
                # Try to switch the pump off before disconnecting.
                abb.send_and_wait(rrc.SetDigital(PUMP_OUTPUT, 0))
                ros.close()
        except Exception as close_error:
            print("Could not safely close ROS:", close_error)