from farmbot import Farmbot, FarmbotToken
import threading
import time
from kinematics import kinematics2D, kinematics3D
import numpy as np
import subprocess
import json

MAX_X, MAX_Y, MAX_Z = 2722.2, 1230, 431.64

class MyHandler:
    def __init__(self, bot):
        # Maintain a flag that lets us know if the bot is
        # ready for more commands.
        self.busy = False
        self.lua_busy = False
        self.lua_id = ""
        self.bot = bot
        self.pos_save = ()
        self.wrist_pin = 6
        self.elbow_pin = 5

    # --------- HANDLER ACTIONS START ----------------
    
    
    # The `on_connect` event is called whenever the device
    # connects to the MQTT server. You can place initialization
    # logic here.
    #
    # The callback is passed a FarmBot instance, plus an MQTT
    # client object (see Paho MQTT docs to learn more).

    def on_connect(self, bot, mqtt_client):
        # Once the bot is connected, we can send RPC commands.
        # Every RPC command returns a unique, random request
        # ID. Later on, we can use this ID to track our commands
        # as they succeed/fail (via `on_response` / `on_error`
        # callbacks):
        print("connected to farmbot")
        self.bot.read_status()


    def on_change(self, bot, state):
        is_busy = state['informational_settings']['busy']
        # print(is_busy)
        if is_busy != self.busy:
            # if is_busy:
            #     print("Device is busy")
            # else:
            #     print("Device is ready to do some action")
            #     self.pos_save = self.bot.position()

            if not is_busy:
                self.pos_save = self.bot.position()


        self.busy = is_busy
        # The `on_change` event is most frequently triggered
        # event. It is called any time the device's internal
        # state changes. Example: Updating X/Y/Z position as
        # the device moves across the garden.
        # The bot maintains all this state in a single JSON
        # object that is broadcast over MQTT constantly.
        # It is a very large object, so we are printing it
        # only as an example.
        # print("NEW BOT STATE TREE AVAILABLE:")
        # print(state)
        # Since the state tree is very large, we offer
        # convenience helpers such as `bot.position()`,
        # which returns an (x, y, z) tuple of the device's
        # last known position:
        # print("Current position: (%.2f, %.2f, %.2f)" % bot.position())
        # A less convenient method would be to access the state
        # tree directly:
        # pos = state["location_data"]["position"]
        # xyz = (pos["x"], pos["y"], pos["z"])
        # print("Same information as before: " + str(xyz))

    # The `on_log` event fires every time a new log is created.
    # The callback receives a FarmBot instance, plus a JSON
    # log object. The most useful piece of information is the
    # `message` attribute, though other attributes do exist.
    def on_log(self, bot, log):
        print("New message from FarmBot: " + log['message'])
        pass

    # When a response succeeds, the `on_response` callback
    # fires. This callback is passed a FarmBot object, as well
    # as a `response` object. The most important part of the
    # `response` is `response.id`. This `id` will match the
    # original request ID, which is useful for cross-checking
    # pending operations.
    def on_response(self, bot, response):
        # print("ID of successful request: " + response.id)
        if response.id == self.lua_id:
            self.lua_busy = False

    # If an RPC request fails (example: stalled motors, firmware
    # timeout, etc..), the `on_error` callback is called.
    # The callback receives a FarmBot object, plus an
    # ErrorResponse object.
    def on_error(self, bot, response):
        # Remember the unique ID that was returned when we
        # called `move_absolute()` earlier? We can cross-check
        # the ID by calling `response.id`:
        print("ID of failed request: " + response.id)
        # We can also retrieve a list of error message(s) by
        # calling response.errors:
        print("Reason(s) for failure: " + str(response.errors))

    # --------- HANDLER ACTIONS END ----------------


    # --------- SELF-DEFINED FUNCTIONS START ----------------

    def wait_till_free(self):
        time.sleep(1)
        while self.busy or self.lua_busy:
            pass
        time.sleep(1)


    def extract_plant_data(self):
         # Parse JSON file from FarmBot into a dictionary
        json_data = handler.parse_json_file('export.json')
        all_points = json_data.get("points")

        plant_list = []

        for small_dict in all_points:
            if small_dict.get("pointer_type") == "Plant":
                plant_list.append((small_dict.get("x"), small_dict.get("y"), small_dict.get("z")))


        return plant_list


    def parse_json_file(self, file_path):
        try:
            with open(file_path, 'r') as file:
                # Load JSON data from file into a dictionary
                data_dict = json.load(file)
                return data_dict
        except FileNotFoundError:
            print("File not found:", file_path)
            return None
        except json.JSONDecodeError as e:
            print("Error decoding JSON:", e)
            return None


    def safe_elbow_flip_height(self):
        self.wait_till_free()
        x, y, z = self.bot.position()
        if z < -292:
            self.bot.move_absolute(x, y, -292, speed=100)
        print("Moving to safe flip height")


    def move_servo(self, wrist_angle, elbow_angle):
        self.wait_till_free()
        # Flip the elbow servo to let the wrist servo go beyond 180
        if wrist_angle > 180:
            wrist_angle = wrist_angle - 180
            elbow_angle = 180 - elbow_angle

        # Map the theoretical servo angles to the actual commands sent
        wrist_actual = ((((180 - wrist_angle) / 180) * (160 - 40) ) + 40)
        elbow_actual = ((((180 - elbow_angle) / 180) * (150 - 30) ) + 30)

        self.bot.set_servo_angle(self.wrist_pin, wrist_actual)        
        self.bot.set_servo_angle(self.elbow_pin, elbow_actual)
        # print("Set wrist servo to angle {0}, elbow servo to angle {1}".format(wrist_angle, elbow_angle))


    def move_xyz(self, x, y, z, speed = 100):
        self.wait_till_free()

        if y > MAX_Y:
            self.bot.move_absolute(x, MAX_Y, z, speed=speed)
        elif x > MAX_X:
            self.bot.move_absolute(MAX_X, y, z, speed=speed)
        elif z > MAX_Z:
            self.bot.move_absolute(x, y, MAX_Z, speed=speed)
        else:
            self.bot.move_absolute(x, y, z, speed=speed)
        # print("Moving to x:{0} and y: {1} and z: {2}".format(x, y, z))


    def move_safe_xyz(self, x, y, z):
        self.wait_till_free()
        print("In move safe xyz")
        
        x_old, y_old, z_old = self.bot.position()
        
        # set z to 0
        self.move_xyz(x_old, y_old, 0)

        # set y to min/max
        if y < 1230/2: #in the first half
            self.move_xyz(x_old, 0, 0)
            self.move_xyz(x, 0, 0)
        else:
            self.move_xyz(x_old, 1230, 0)
            self.move_xyz(x, 1230, 0)

        # move y to y
        self.move_xyz(x, y, 0)

        # move z to z
        self.move_xyz(x, y, z)

        print("Done with safe movement")


    def take_photo_uart(self, reversed = False):
        # bot.lua("move{x=20, y=0, z=-100}")
        # bot.lua("""
        #     uart_list = uart.list()

        #     for _number, device_path in ipairs(uart_list) do
        #         toast(inspect(device_path), "debug")
        #     end
        #     """)
        self.wait_till_free()
        self.lua_busy = True

        if reversed:
            self.lua_id = self.bot.lua("""
                toast("start LUA", "debug")
                    
                -- device name, baud rate:
                my_uart, error = uart.open("ttyS0",115200)
                
                if error then
                    toast("couldn't open uart", "error")
                    return
                end
                    
                if my_uart then
                    error3 = my_uart.write("U")

                    if error3 then
                        -- Handle errors etc..
                        toast("failed to send data", "error")
                    else
                        toast("Data sent", "debug")
                    end

                    -- Wait 40s for data...
                    response, error2 = my_uart.read(40000)

                    if error2 then
                        toast("failed to receive data", "error")
                    else
                        toast("Received: " .. response)
                    end

                    if response == "Y" then
                        toast("took a picture", "debug") 
                    else
                        toast("no picture taken", "debug")  
                        
                    end       
                    my_uart.close()
                end
                toast("end LUA", "debug")
                """)
        else:
            self.lua_id = self.bot.lua("""
            toast("start LUA", "debug")
                
            -- device name, baud rate:
            my_uart, error = uart.open("ttyS0",115200)
            
            if error then
                toast("couldn't open uart", "error")
                return
            end
                
            if my_uart then
                error3 = my_uart.write("P")

                if error3 then
                    -- Handle errors etc..
                    toast("failed to send data", "error")
                else
                    toast("Data sent", "debug")
                end

                -- Wait 40s for data...
                response, error2 = my_uart.read(40000)

                if error2 then
                    toast("failed to receive data", "error")
                else
                    toast("Received: " .. response)
                end

                if response == "Y" then
                    toast("took a picture", "debug") 
                else
                    toast("no picture taken", "debug")  
                    
                end       
                my_uart.close()
            end
            toast("end LUA", "debug")
            """)


    def go_around_plant(self, location, radius):
        # Resume from location
        check_z = -260  #Is the min z (-300)
        check_wrist_angle = 120  #0 is original
        check_elbow_angle = 0  #0 is original
        # Resume from location

        min_wrist_angle = 0
        max_wrist_angle = 360
        wrist_step_degree = 30            #EDIT

        min_elbow_angle = 0
        max_elbow_angle = 16
        elbow_step_degree = 15            #EDIT

        x_plant_center, y_plant_center, z_plant_center = location

        min_z = -300
        max_z = z_plant_center                
        step_z = 20                 #EDIT
        
        # Move to center of plant circle
        # self.move_xyz(x_plant_center, y_plant_center, z_plant_center)          #TODO

        # Take a top down img               #TODO
        # print("Going to the plant location")
        # self.move_servo(5, 90)
        # self.take_photo_uart(self.bot)
        # time.sleep(3)

        # Go to starting position
        x, y = kinematics2D(r=radius, plant_x=x_plant_center, plant_y=y_plant_center, angle=0)  # TODO

        x0, y0, z0 = self.bot.position()
        print("Plant will go to starting position at {0},{1},{2}".format(x, y, z0))

        self.move_xyz(x, y, 0)
        self.move_servo(6, 0)
        # self.move_xyz(x, y, min_z)

        # Revolve aroud plant and point camera up and down
        start = True
        for z_height in range(min_z, max_z, step_z):

            if z_height < check_z:
                continue

            for wrist_angle in range(min_wrist_angle, max_wrist_angle, wrist_step_degree):

                if wrist_angle < check_wrist_angle and start:
                    continue
                
                if wrist_angle == (180 + wrist_step_degree) or wrist_angle == 0:
                    self.safe_elbow_flip_height()
                    
                for elbow_angle in range(min_elbow_angle, max_elbow_angle, elbow_step_degree):

                    if elbow_angle < check_elbow_angle and start:
                        continue

                    # if z_height == min_z and elbow_angle > 0:
                    #    continue

                    if elbow_angle == 15:
                        if z_height == -300:
                            self.move_servo(wrist_angle, elbow_angle)
                            x, y = kinematics2D(r=radius, plant_x=x_plant_center, plant_y=y_plant_center, angle=wrist_angle)
                            if start:
                                # self.move_safe_xyz(x, y, z_height)
                                start = False
                            else:
                                self.move_xyz(x, y, z_height)

                            # take a photo
                            if wrist_angle > 180:
                                self.take_photo_uart(reversed=True)
                            else:
                                self.take_photo_uart()
                        else:
                            continue
                    else:
                        self.move_servo(wrist_angle, elbow_angle)
                        x, y = kinematics2D(r=radius, plant_x=x_plant_center, plant_y=y_plant_center, angle=wrist_angle)
                        if start:
                            # self.move_safe_xyz(x, y, z_height)
                            start = False
                        else:
                            self.move_xyz(x, y, z_height)

                        # take a photo
                        if wrist_angle > 180:
                            self.take_photo_uart(reversed=True)
                        else:
                            self.take_photo_uart()

                print("Current State of Robot: z_height {0}, wrist_angle: {1}".format(z_height, wrist_angle))
            
            print(f"Height of z {z_height} done")

        # Move the robot back to starting position
        x, y = kinematics2D(r=radius, plant_x=x_plant_center, plant_y=y_plant_center, angle=0)
        self.move_xyz(x, y, z_plant_center)
        print("Cylinder done")

    
    def go_above_plant(self, location, radius):
        min_theta = 0
        max_theta = 360
        theta_step = 20            #EDIT

        phi_step = 30            #EDIT
        min_phi = -1
        max_phi = 90-phi_step

        x_plant_center, y_plant_center, z_plant_center = location
    
        for phi in range(max_phi, min_phi, -phi_step):
            for theta in range(min_theta, max_theta, theta_step):
                self.move_servo(theta, 90-phi)
                x, y, z = kinematics3D(r=radius, plant_x=x_plant_center, plant_y=y_plant_center, plant_z=z_plant_center, theta=theta, phi=phi)
                self.move_xyz(x, y, z)
                
                # Take a photo
                if theta > 180:
                    self.take_photo_uart(reversed=True)
                else:
                    self.take_photo_uart()

                print("Current State: phi: {0}, theta: {1}".format(phi, theta))

        # Move the robot back to starting position
        x, y = kinematics2D(r=radius, plant_x=x_plant_center, plant_y=y_plant_center, angle=0)
        self.move_xyz(x, y, 0)
        print(f"Hemisphere done")


    def go_between_plants(self, old_plant_loc, old_radius, next_plant_loc, next_radius):
        # version for max y instead

        # Move to y = 0
        # self.move_xyz(old_plant_loc[0] + old_radius, 0, 0, speed=100)
        self.move_xyz(old_plant_loc[0] + old_radius, 1300, 0, speed=100) # max y instead


        # Move x = new_plant_x - old_plant_x
        # self.move_xyz(next_plant_loc[0] + next_radius, 0, 0)
        self.move_xyz(next_plant_loc[0] + next_radius, 1300, 0)


        # Move to y = next_plant_loc[y]
        self.move_xyz(next_plant_loc[0] + next_radius, next_plant_loc[1], 0)

        print("Moving to the new plant")

    def go_to_all_plants(self):
        self.wait_till_free()
        #all_plant_loc = self.extract_plant_data()[::-1][2:]
        # [x-axis, y-axis, z-max height]
        all_plant_loc = [(331, 771, -150)] #110 is the arm length until the wrist
        all_plant_radius = [500]
        old_loc = [0, 0, 0]

        for i, plant_loc in enumerate(all_plant_loc):
            self.go_between_plants(old_loc, all_plant_radius[i-1], plant_loc, all_plant_radius[i])
            time.sleep(2)
            print(f"Ready for plant {i}, going around plant")
            self.go_around_plant(plant_loc, all_plant_radius[i])
            print("Going above the plant now\n\n")
            self.go_above_plant(plant_loc, all_plant_radius[i])
            old_loc = plant_loc

    def test_uart(self):
        self.wait_till_free()
        print("Testing UART Connection")
        self.lua_busy = True

        self.lua_id = self.bot.lua("""
            toast("start LUA", "debug")
                
            -- device name, baud rate:
            my_uart, error = uart.open("ttyS0",115200)
            
            if error then
                toast("couldn't open uart", "error")
                return
            end
                
            if my_uart then
                error3 = my_uart.write("P")

                if error3 then
                    -- Handle errors etc..
                    toast("failed to send data", "error")
                else
                    toast("Data sent", "debug")
                end

                -- Wait 15s for data...
                response, error2 = my_uart.read(6000)

                if error2 then
                    toast("failed to receive data", "error")
                else
                    toast("Received: " .. response)
                end

                if response == "Y" then
                    toast("UART Connection Successful", "debug") 
                else
                    toast("UART Connection Error", "debug")  
                    
                end       
                my_uart.close()
            end
            toast("end LUA", "debug")
            """)
        
    def initialization_sequence(self):

        # Go back home
        self.bot.find_home()

        while True:
            # Test UART and camera
            self.test_uart()
            time.sleep(2)
            ipt = input("Did test_uart run? Enter 'Y' for Yes, or any other key to try again: ")

            if ipt.upper() == "Y":
                print("UART Works!")
                break
            else:
                print("UART not working. Trying again")

        print("UART Testing Successful")

        while True:

            for wrist in range(0, 180, 90):
                for elbow in range(0, 90, 45):
                    self.move_servo(wrist, elbow)
                    print("Moved Servo")

            print("Servo testing done\n\n")
            ipt = input("Did servos run? Enter 'Y' for Yes, or any other key to try again: ")

            if ipt.upper() == "Y":
                print("Servos Testing Successful")
                break
            else:
                print("Servos not working. Trying again")

        print("Inilization Done")

    def test_seq(self):
        i = True
        while True:
            # self.test_uart()
            self.take_photo_uart()
            if i:
                print("Moving to 0")
                self.move_xyz(0, 0, 0)
                i = False
            else:
                print("Moving to 150")
                self.move_xyz(150, 150, 0)
                i = True

    def total_time():
        # time taken for one image ~26sec
        time_img = 26
        elbow_step_degree = 360 #degrees
        elbow_max = 360
        elbow_min = 0

        wrist_step_degree = 5
        
        min_theta = 0
        max_theta = 360
        theta_step = 5
        
        min_z = -320               
        max_z = -150                
        step_z = 20  

        phi_step = 20
        min_phi = -1
        max_phi = 90-phi_step

        # time for the cylinder
        # number of angles gone
        angles_cnt = np.floor((elbow_max - elbow_min)/elbow_step_degree) * np.floor(360/wrist_step_degree) * np.floor((max_z - min_z)/step_z)

        angles_dome = np.floor((max_theta - min_theta) / theta_step) * np.floor((max_phi - min_phi) / phi_step)

        total_images = (angles_cnt + angles_dome) #in sec

        seconds = total_images * time_img #in sec

        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        remaining_seconds = seconds % 60

        print("Total number of images ", total_images)
        print("Time for this plant: {0} hours {1} minutes {2} seconds".format(hours, minutes, remaining_seconds))


    # --------- SELF-DEFINED FUNCTIONS end ----------------


if __name__ == '__main__':
    fb = Farmbot.login(email="vmuriki3@gatech.edu",
                   password="Qwerty@123",
                   server="https://my.farm.bot")
    handler = MyHandler(fb)
    threading.Thread(target=fb.connect, name="firstbot", args=[handler], daemon=True).start()
    time.sleep(1)

    handler.initialization_sequence()
    handler.go_to_all_plants()
    # handler.test_seq()
    # handler.move_xyz(2000, 0,0)

    # while True:
    #     print(handler.busy)
    
    while handler.lua_busy:
        pass