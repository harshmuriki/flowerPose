from farmbot import Farmbot
import threading
import time
from kinematics import kinematics2D
import time
import numpy as np
# import opencv as cv2

class MyHandler:
    def __init__(self, bot):
        # Maintain a flag that lets us know if the bot is
        # ready for more commands.
        self.busy = True
        self.bot = bot

    def move_servo(self, pin, angle):
        time.sleep(0.5)
        if angle > 180:
            self.bot.set_servo_angle(5, 30)
            angle = angle - 180
        else:
            self.bot.set_servo_angle(5, 150)

        new_angle = ((((180 - angle) / 180) * (160 - 40) ) + 40)
        self.bot.set_servo_angle(pin, new_angle)
        print("Set servo {0} to angle {1}".format(pin, angle))

    def move_xyz(self, x, y, z, speed = 70):

        while self.busy:
            # print("Bot moving")
            pass
        
        print("Doing the next angle")
        time.sleep(0.5)
        id = self.bot.move_absolute(x, y, z, speed=speed)
        # print("HERE:", id)

    def move_xyz_adv(self, x, prev_x, y, prev_y, z, speed = 70):

        while self.busy:
            # print("Bot moving")
            pass

        # prev_x, prev_y, prev_z = self.bot.position()

        x_smooth = np.linspace(prev_x, x, num = 5)
        y_smooth = np.linspace(prev_y, y, num = 5)

        # print("x", prev_x, x)
        # print("y", prev_y, y)

        print("x", x_smooth)
        print("y", y_smooth)

        for i in range(len(x_smooth)):
            x_s = x_smooth[i]
            y_s = y_smooth[i]
            time.sleep(0.5)
            self.bot.move_absolute(x_s, y_s, z, speed=speed)


    def on_connect(self, bot, mqtt_client):
        self.bot.read_status()
        pass

    def on_change(self, bot, state):
        is_busy = state['informational_settings']['busy']
        if is_busy != self.busy:
            if is_busy:
                print("Device is busy")
            else:
                print("Device is ready to do some action")

        self.busy = is_busy

    def on_log(self, _bot, log):
        # print("LOG: " + log['message'])
        pass

    def on_response(self, _bot, _response):
        # print(response)
        pass
        
    def on_error(self, _bot, response):
        print("ERROR: " + response.id)
        print("Reason(s) for failure: " + str(response.errors))

    def safe_z(self):
        x, y, z = self.bot.position()
        self.bot.move_relative(x, y, 0, speed=100)
        time.sleep(5)
        while self.busy:
            pass
        print("Safe z set")

    def go_around_plant(self):

        try:
            min_angle = 0
            max_angle = 360
            step_degree = 45

            min_z = -270
            max_z = -199
            step_z = 30

            self.safe_z()

            prev_x, prev_y = None, None

            self.move_xyz(560, 960, 0)

            # take a top down img
            self.move_servo(5, 90)
            self.bot.take_photo()
            print("Going to the plant location")
            time.sleep(3)

            for z_height in range(min_z, max_z, step_z):

                for temp_angle in range(min_angle, max_angle, step_degree):
                    x, y = kinematics2D(r=200, plant_x=560, plant_y=960, angle=temp_angle)
                    
                    self.move_xyz(x, y, z_height)

                    # if prev_x is None:
                        # self.move_xyz(x, y, z_height)
                    # else:
                        # self.move_xyz_adv(x, prev_x, y, prev_y, z_height)

                    # rotate arm
                    self.move_servo(6, temp_angle)

                    # take a photo
                    self.bot.take_photo()

                    prev_x, prev_y = x, y

                    print("Moved to x:{0} and y: {1} and z: {2}".format(x, y, z_height))

                print("Height of z {0} done".format(z_height))

            print("Done!")

            exit()

        except KeyboardInterrupt:
            print("Exiting")

        print("Exited")

    def send_data_UART(self, bot):

        bot.lua("""
            toast("start LUA", "debug")
               
            -- device name, baud rate:
            my_uart, error = uart.open("ttyS0", 115200)
           
            if error then
                toast(inspect(error), "error")
                return
            end
               
            if my_uart then
                toast("UART connected", "debug")
 
                error3 = my_uart.write(string.byte("Hello, world! it is working!!!!"))
                wait(1000)
                toast("writing...","debug")
               
                if error3 then
                    -- Handle errors etc..
                    toast(inspect(error3), "error")
                end
 
                my_uart.close()
            end
           
            toast("end LUA", "debug")
            """
                )


if __name__ == '__main__':
    f = open("raw_token.txt", "r")
    data = f.read()
    raw_token = data.encode('utf-8')
    fb = Farmbot(raw_token)  #fb --> bot
    handler = MyHandler(fb)
    threading.Thread(target=fb.connect, name="firstbot", args=[handler]).start()

    # handler.go_around_plant()
    handler.send_data_UART(fb)
