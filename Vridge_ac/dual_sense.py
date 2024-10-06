from time import sleep

from dualsense_controller import DualSenseController

# list availabe devices and throw exception when tzhere is no device detected
device_infos = DualSenseController.enumerate_devices()
if len(device_infos) < 1:
    raise Exception('No DualSense Controller available.')

# flag, which keeps program alive
is_running = True

# create an instance, use fiŕst available device
controller = DualSenseController()


# switches the keep alive flag, which stops the below loop
def stop():
    global is_running
    is_running = False


# callback, when cross button is pressed, which enables rumble
def on_cross_btn_pressed():
    print('cross button pressed')
    controller.left_rumble.set(255)
    controller.right_rumble.set(255)


# callback, when cross button is released, which disables rumble
def on_cross_btn_released():
    print('cross button released')
    controller.left_rumble.set(0)
    controller.right_rumble.set(0)


# callback, when PlayStation button is pressed
# stop program
def on_ps_btn_pressed():
    print('PS button released -> stop')
    stop()


# callback, when unintended error occurs,
# i.e. physically disconnecting the controller during operation
# stop program
def on_error(error):
    print(f'Opps! an error occured: {error}')
    stop()


# register the button callbacks
controller.btn_cross.on_down(on_cross_btn_pressed)
controller.btn_cross.on_up(on_cross_btn_released)
controller.btn_ps.on_down(on_ps_btn_pressed)

# register the error callback
controller.on_error(on_error)

# enable/connect the device
controller.activate()

def on_battery_change(battery) -> None:
    print(f'on battery change: {battery}')


def on_battery_lower_than(battery_level) -> None:
    print(f'on battery low: {battery_level}')


def on_battery_charging(battery_level) -> None:
    print(f'on battery charging: {battery_level}')


def on_battery_discharging(battery_level) -> None:
    print(f'on battery discharging: {battery_level}')


controller.battery.on_change(on_battery_change)
controller.battery.on_lower_than(20, on_battery_lower_than)
controller.battery.on_charging(on_battery_charging)
controller.battery.on_discharging(on_battery_discharging)


def on_cross_btn_pressed():
    print('cross button pressed')


def on_cross_btn_released():
    print('cross button_released')


def on_cross_btn_changed(pressed):
    print(f'cross button is pressed: {pressed}')


controller.btn_cross.on_down(on_cross_btn_pressed)
controller.btn_cross.on_up(on_cross_btn_released)
controller.btn_cross.on_change(on_cross_btn_changed)


def on_left_trigger(value):
    print(f'left trigger changed: {value}')


def on_left_stick_x_changed(left_stick_x):
    print(f'on_left_stick_x_changed: {left_stick_x}')


def on_left_stick_y_changed(left_stick_y):
    print(f'on_left_stick_y_changed: {left_stick_y}')


def on_left_stick_changed(left_stick):
    print(f'on_left_stick_changed: {left_stick}')


controller.left_trigger.on_change(on_left_trigger)
controller.left_stick_x.on_change(on_left_stick_x_changed)
controller.left_stick_y.on_change(on_left_stick_y_changed)
controller.left_stick.on_change(on_left_stick_changed)

def on_gyroscope_change(gyroscope):
    print(f'on_gyroscope_change: {gyroscope}')


def on_accelerometer_change(accelerometer):
    print(f'on_accelerometer_change: {accelerometer}')


def on_orientation_change(orientation):
    print(f'on_orientation_change: {orientation}')


controller.gyroscope.on_change(on_gyroscope_change)
controller.accelerometer.on_change(on_accelerometer_change)
controller.orientation.on_change(on_orientation_change)

controller.lightbar.set_color(88, 10, 200)

# controller.left_trigger.effect.continuous_resistance(start_position=0, force=255)  # full resistance
# controller.left_trigger.effect.continuous_resistance(start_position=127, force=255) # full resist. starts at middle pos
# controller.left_trigger.effect.continuous_resistance(start_position=0, force=128)  # medium resistance

# controller.left_trigger.effect.bow(start_position=1, end_position=4, strength=1, snap_force=8) 

# start keep alive loop, controller inputs and callbacks are handled in a second thread
while is_running:
    sleep(0.001)

# disable/disconnect controller device

controller.left_trigger.effect.no_resistance()
controller.deactivate()
