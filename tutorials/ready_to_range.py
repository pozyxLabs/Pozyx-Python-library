from pypozyx import *

port = '/dev/ttyACM0'

remote = False
remote_id = 0x605D

if not remote:
    remote_id = None

destination_id = 0x1000
range_step_mm = 1000


class ReadyToRange():

    def __init__(self, port):
        try:
            self.pozyx = PozyxSerial(port)
        except:
            print('ERROR: Unable to connect to Pozyx, wrong port')
            raise SystemExit

        self.setup()
        while True:
            self.loop()

    def setup(self):
        print("------------POZYX RANGING V1.0 - -----------\nNOTES: \n - Change the parameters: \n\tdestination_id(target device)\n\trange_step(mm)\n- Approach target device to see range and\nled control\n- -----------POZYX RANGING V1.0 ------------\nSTART Ranging: ")
        led_config = 0x0
        # make sure the local/remote pozyx system has no control over the LEDs.
        self.pozyx.setLedConfig(led_config, remote_id)
        # do the same for the destination.
        self.pozyx.setLedConfig(led_config, destination_id)

    def loop(self):
        device_range = DeviceRange()
        status = self.pozyx.doRanging(destination_id, device_range, remote_id)
        if status == POZYX_SUCCESS:
            print(device_range)
            self.ledControl(device_range.distance)
        else:
            print("ERROR: ranging")

    def ledControl(self, distance):
        status = POZYX_SUCCESS
        ids = [remote_id, destination_id]
        # set the leds of both local/remote and destination pozyx device
        for id in ids:
            status &= self.pozyx.setLed(4, (distance < range_step_mm), id)
            status &= self.pozyx.setLed(3, (distance < 2 * range_step_mm), id)
            status &= self.pozyx.setLed(2, (distance < 3 * range_step_mm), id)
            status &= self.pozyx.setLed(1, (distance < 4 * range_step_mm), id)
        return status

if __name__ == "__main__":
    r = ReadyToRange(port)
