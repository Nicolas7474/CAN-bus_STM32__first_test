# CAN bus interface CAN bus 2.0A Monitoring Interface (working draft, project for STM32F469)

import dearpygui.dearpygui as dpg
import errno # The errno module maps error numbers to symbolic names for OS-related errors.
import CAN_Stm32_GUI  # Import our second file
import serial                    # Import the UART communication library
import serial.tools.list_ports
import threading                 # Import threading to run UART and GUI at the same time
import time
from datetime import datetime # Import time for delays and sleep
import tkinter as tk # only to auto-center the dpg window
import queue  # Thread-safe queue
import re
from cobs import cobs

WIDTH_DPG_WIN = 800
HEIGHT_DPG_WIN = 700
PADDING_R_LIST_PORTS = 50
CAN_SPEEDS = ["1000 kbps", "800 kbps", "500 kbps", "250 kbps", "125 kbps", "100 kbps", "50 kbps"] # Common speeds
BAUD = 115200       # Bits per second; must match your STM32 HAL_UART_Init

class STM32Monitor:
   
    def __init__(self):   
        dpg.create_context()  
        self.running = False       # A flag to control the loops (True = keep running)
        self.handshake_done = False
        self.selected_port = None
        self.thread = None
        self.nb_frames = 0
        self.err_frames = 0       
        self.rx_queue = queue.Queue() 
        self.tx_queue = queue.Queue()
        self.max_rows = 100   # Prevent infinite growth
        self.can_speed_packet = b""    # An empty byte object instead of None to avoid static error

        # Load resources from support.py
        self.fonts = CAN_Stm32_GUI.load_assets()
        self.t_off, self.t_on, self.t_stop = CAN_Stm32_GUI.create_button_themes()
        self.table_theme = CAN_Stm32_GUI.theme_table()
        self.error_theme = CAN_Stm32_GUI.theme_error()
    
        # Viewport autocenter: get screen size
        root = tk.Tk()
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        root.destroy() # This completely removes the Tkinter window and cleans up resources   
         
        with dpg.window(tag="primary_window", label="STM32F469"):
            # Header
            with dpg.group(horizontal=True):     
                # Circle red/green Serial Port: using a small drawlist as a container                   
                with dpg.drawlist(width=40, height=30):       
                    dpg.draw_circle((30, 15), 10, fill=[180, 15, 0], tag="cercle")
                dpg.add_spacer(width=5)
                header = dpg.add_text("CAN bus Frames Monitor", color=[245, 255, 200])
                dpg.bind_item_font(header, self.fonts["M"])                              
                dpg.add_spacer(width=60)

                # --- COM ports dropdown menu ---             
                def get_com_number(item):   # Helper function to avoid error when result is None with simple inline instruction
                    # Sort: we split by 'COM', then take the part after it, then split by ' ' (to remove the description) and convert to int
                    match = re.search(r'COM(\d+)', item)  # Logic: "COM17 - Desc" -> "17 - Desc" -> "17" -> 17
                    if match:
                        return int(match.group(1)) 
                    return 0  # Default value if no COM number is found
                def get_port_list():  # list available devices (remove the redondant COMx from the device's description 
                    list = [f"{p.device} - {p.description.replace(f"({p.device})", "").strip()}" for p in serial.tools.list_ports.comports()] 
                    list.sort(key=get_com_number) # sort the list by growing COM number after isolating the port number with Regex                            
                    return list
                def update_ports():
                    if dpg.is_item_enabled("port_dropdown"): # Only scan if we aren't using the port (Worker is disabled)
                        new_list = get_port_list()
                        displayed_value = dpg.get_value("port_dropdown")                        
                        # Only update the COM port dropdown if the list is actually different to prevent flickering on hovering
                        if dpg.get_item_configuration("port_dropdown")["items"] != new_list: # Check if hardware was added or removed
                            dpg.configure_item("port_dropdown", items=new_list)
                            if displayed_value not in new_list: # Reset the label of first row to clear the old value
                                dpg.set_value("port_dropdown", "Select a COM port") 
                                self.selected_port = None 
                            if not new_list: # if the list is now empty
                                dpg.set_value("port_dropdown", "No Ports Found")                            
                                self.selected_port = None # Reset the internal variable so start_worker knows no port is chosen          
                dpg.add_combo(items=get_port_list(), default_value="Select a COM port", tag="port_dropdown", 
                              pos = (400, 12), fit_width=True, callback=lambda s, a: (self.on_port_select(s, a), self.align_dropdown(s)))                  
                # To refresh the port list without any button, the list is updated when hovering on the menu box
                # We must create an "item_handler_registry", it is like a listener that waits for the hover event
                with dpg.item_handler_registry(tag="port_refresh_handler"):
                    dpg.add_item_hover_handler(callback=update_ports) 
                dpg.bind_item_handler_registry("port_dropdown", "port_refresh_handler")  
                                       
            dpg.add_separator()            
            dpg.add_spacer(height=15)      
         
            with dpg.group(horizontal=True):
                dpg.add_spacer(width=10)
                # Create the table
                with dpg.child_window(width=475, height=430):
                    with dpg.table(tag="can_table_disp", header_row=True, resizable=False, policy=dpg.mvTable_SizingFixedFit,
                    row_background=True, borders_innerV=True, borders_outerV=True, 
                    borders_innerH=True, borders_outerH=True,
                    scrollX=False, scrollY=False, width=452, height=412) as can_table:
                    
                        dpg.bind_item_theme(can_table, self.table_theme)
                        dpg.bind_item_font(can_table, self.fonts["S"])
                        # Create Columns (matching your image)
                        col_time = dpg.add_table_column(label="Time", init_width_or_weight=85)
                        dpg.bind_item_font(col_time, self.fonts["M"]) 
                        dpg.add_table_column(label="Id", init_width_or_weight=42)
                        dpg.add_table_column(label="RTR", init_width_or_weight=27)  # RTR  
                        dpg.add_table_column(label="Len", init_width_or_weight=25) 
                        dpg.add_table_column(label="", init_width_or_weight=0)                                      
                        for i in range(8):
                            dpg.add_table_column(label=f"D{i}", init_width_or_weight=19)   

                dpg.add_spacer(width=5)   
                # Right hand side informations
                with dpg.group(width=130, height=20):                      
                    dpg.add_text("Number of frames", color=[245, 255, 200])
                    with dpg.group(horizontal=True):
                        dpg.add_text("received:", color=[245, 255, 200])
                        dpg.add_text("0", tag="nb_frames", color=[245, 255, 200])
                    dpg.add_spacer(height=5)
                    dpg.add_text("Python discarded frames", color=[245, 255, 200])
                    with dpg.group(horizontal=True):
                        dpg.add_text("(with errors):", color=[245, 255, 200])
                        dpg.add_text("0", tag="error_frames", color=[245, 255, 200])
                    dpg.add_spacer(height=5)
                    dpg.add_text("State of handshake (CAN speed Ack):", color=[245, 255, 200])
                    dpg.add_text("-", tag="state_handshake1", color=[179, 223, 252])
                    dpg.add_text("", tag="state_handshake2", color=[255, 127, 107])

                    dpg.add_spacer(height=20)
                    # CAN addresses Filter
                    dpg.add_text("Hardware Filters", color=[245, 255, 200])
                    with dpg.group(horizontal=True):
                        dpg.add_text("ID:   0x")
                        dpg.add_input_text(tag="f_id", width=60, default_value="000", no_spaces=True, uppercase=True)
                    with dpg.group(horizontal=True):
                        dpg.add_text("Mask: 0x")
                        dpg.add_input_text(tag="f_mask", width=60, default_value="000", no_spaces=True, uppercase=True)
                    dpg.add_button(label="Apply Filter", width=130, callback=self.send_filter_config)
                    dpg.add_text("0x000 & 0x000 = Receive All", color=[190, 190, 190])
                    dpg.add_text("Filter settings applied !", tag="filter_acked", color=[255, 127, 107], show=False)

            dpg.add_spacer(height=50)                
          
            # Bottom Control Buttons  
            with dpg.group(horizontal=True):               
                dpg.add_spacer(width=10)
                # Startp_Stop button
                start_btn = dpg.add_button(
                    label="Stopped", tag="start_button_tag", width=100, height=26,                                        
                    user_data=[self, "cercle", "port_dropdown", "can_speed_dropdown"], callback=CAN_Stm32_GUI.toggle_start_handler)   
                dpg.bind_item_font(start_btn, self.fonts["S"])
                dpg.bind_item_theme(start_btn, self.t_off)
                dpg.add_spacer(width=15)
                # Clear button
                clear_btn = dpg.add_button( label="Clear",   width=60, height=26, callback=self.clear_table)
                dpg.bind_item_font(clear_btn, self.fonts["S"])
                dpg.bind_item_theme(clear_btn, self.t_stop)
                dpg.add_spacer(width=15)              
                # Send button
                send_btn = dpg.add_button(label="Test Tx", width=60, height=26, callback=self.send_command)
                dpg.bind_item_font(send_btn, self.fonts["S"])
                dpg.bind_item_theme(send_btn, self.t_stop)
                dpg.add_spacer(width=15)
                # Send baudrate dropdown
                with dpg.group(pos=[340, 540]): 
                    dpg.add_text("CAN Bus Speed:")
                with dpg.group(pos=[340, 560]):                            
                    # Send a UART command to STM32 to change CAN speed                        
                    dpg.add_combo(items=CAN_SPEEDS, default_value="250 kbps", tag="can_speed_dropdown", width=150, callback=self.update_can_speed) 

        # alert window, Serial Port failed to connect
        with dpg.window(tag="error_modal", label="Error", 
                        modal=True, show=False, 
                        no_title_bar=True, no_move=True, no_resize=True,
                        pos=(150, 200), width=300):
            dpg.add_text("SERIAL PORT ERROR", color=(250, 160, 160))
            dpg.add_separator()           
            dpg.add_text("", tag="error_message_text", color=(255, 255, 235))
            dpg.bind_item_font("error_message_text", self.fonts["S"]) 
            dpg.add_text("Check connection.", tag="error_messagechk_text", color=(255, 255, 235)), # type: ignore
            dpg.bind_item_font("error_messagechk_text", self.fonts["XS"]) 
            dpg.add_spacer(height=5)
            with dpg.group(horizontal=True):
                dpg.add_spacer(width=190)
                dpg.add_button(label="OK", width=75, callback=lambda: dpg.configure_item("error_modal", show=False))

        with dpg.handler_registry():           
            dpg.add_key_release_handler(key=dpg.mvKey_Escape, callback=self.stop_all)

        pos_x = int((screen_width - WIDTH_DPG_WIN) / 2)
        pos_y = int((screen_height - HEIGHT_DPG_WIN) / 2)        
        dpg.create_viewport(title='STM32 CAN 2.0A Monitor',
                            width=WIDTH_DPG_WIN, height=HEIGHT_DPG_WIN, max_width=WIDTH_DPG_WIN, max_height=HEIGHT_DPG_WIN, 
                            x_pos=pos_x, y_pos=pos_y, resizable=False)        
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("primary_window", True)  # When a window is set as primary, "True" will always make it match the viewport size.
    
        dpg.set_frame_callback(1, callback=self.align_dropdown)
    
    def align_dropdown(self, sender=None):                  
        dpg.split_frame()  # forces layout update
        # Use self. to access class attributes        
        width = dpg.get_item_rect_size("port_dropdown")[0]       
        x_start_pos = WIDTH_DPG_WIN - width - PADDING_R_LIST_PORTS # calculate the x_pos to get constant padding on the right
        dpg.configure_item("port_dropdown", pos=[x_start_pos, 12])        

    def on_port_select(self, sender, app_data):              
        self.selected_port = app_data.split(" ")[0]   # Extract just "COMx" from "COMx - Description" 
        print(f"Port set to: {self.selected_port}")
    
    def update_can_speed(self, sender, app_data):      
        speed_map = {"1000 kbps": 0x00, "800 kbps": 0x01, "500 kbps": 0x02, "250 kbps": 0x03, "125 kbps": 0x04, "100 kbps": 0x05, "50 kbps": 0x06}  
        cmd_byte = speed_map.get(app_data.strip()) # app_data contains the value chosen for ex. "250 kbps"
        if cmd_byte is not None:
            packet = bytes([0x02, 0xCF, cmd_byte])  # Follows the protocol packet (Len = 0x02, Cmd=0xCF, Value)
            encoded_msg = cobs.encode(packet)       # COBS eliminates the need for dedicated start/end bits
            self.can_speed_packet = b'\x00' + encoded_msg + b'\x00'  # Add leading and trailing delimiters

    def send_filter_config(self):        
        try:
            # 1. Grab values from GUI
            val_id = int(dpg.get_value("f_id"), 16) & 0x7FF # Ensure 11-bit limit
            val_mask = int(dpg.get_value("f_mask"), 16) & 0x7FF            
            # 2. Build the packet: [LEN, CMD, ID_H, ID_L, MASK_H, MASK_L]
            # We split the 11-bit IDs into two bytes for the UART
            packet = bytes([
                0x05, 0xAF, 
                (val_id >> 8) & 0xFF, val_id & 0xFF,
                (val_mask >> 8) & 0xFF, val_mask & 0xFF
            ])            
            # 3. Encode and Queue
            encoded = b'\x00' + cobs.encode(packet) + b'\x00'
            self.tx_queue.put(encoded)
            print(f"Sent Filter: ID=0x{val_id:03X}, Mask=0x{val_mask:03X}")            
        except ValueError:
            # If user types something that isn't Hex
            dpg.set_value("state_handshake", "Error: Invalid Hex")
        
    def clear_table(self):
        dpg.delete_item("can_table_disp", children_only=True, slot=1) 

    def start_worker(self):
        if self.thread is None or not self.thread.is_alive():        
            if not self.selected_port:
                self.running = False
                dpg.set_value("error_message_text", "Please select a COM port first !")
                dpg.configure_item("error_modal", show=True) 
                dpg.configure_item("error_messagechk_text", show=False)                
                dpg.bind_item_theme("start_button_tag", self.t_off) # Use the button tag to change the theme to               
                return
            dpg.configure_item("port_dropdown", enabled=False)   
            dpg.configure_item("can_speed_dropdown", enabled=False)
            # Ensure we have a default packet at startup if the user hasn't touched the dropdown       
            current_val = dpg.get_value("can_speed_dropdown")
            self.update_can_speed(None, current_val)  
            # self.running = True is set in the GUI file    
            self.thread = threading.Thread(target=self.read_serial, daemon=True)
            self.thread.start() 

    def blink_logic(self):
        # Loop for 5 seconds (5 cycles of 1 second)
        for _ in range(6):
            dpg.configure_item("filter_acked", show=True)  # Toggle off
            time.sleep(0.7)         
            dpg.configure_item("filter_acked", show=False)  # Toggle on
            time.sleep(0.3)


    def speed_ack_blinking(self):
        # Run in a thread so the GUI doesn't freeze
        threading.Thread(target=self.blink_logic, daemon=True).start()
   
    def read_serial(self):
        """ This function runs in the background thread """
        try:
            with serial.Serial(self.selected_port, BAUD, timeout=0.1, dsrdtr=False) as ser: 
    
                # The port is now OPEN. We can safely send the speed packet.
         
                self.handshake_done = False        # RESET state for new connection
                start_time = time.time() # Track when we started

                if hasattr(self, 'can_speed_packet'):                 
                    ser.write(self.can_speed_packet)
                    # No 'while' loop here anymore!
                        
                buffer = bytearray() # Temporary storage for incoming fragments  
                while self.running:                 
                    # By using if ser.in_waiting == 0: continue, you get the best performance without the risks of low timeouts:
                    # Safety: You can keep a longer timeout (0.1 or 0.5). This ensures that once a frame starts (the 0x02 is found), 
                    # the subsequent read(4) and read(dlc+1) have plenty of time to wait for the remaining bytes to arrive if there is a slight delay.
                    # Speed: You aren't stuck waiting for that timeout when the line is idle. The in_waiting check is near-instant at the hardware driver level.
                    
                    # --- THE SILENT TIMER CHECK ---
                    # This runs every loop, even if no data has arrived yet
                    if not self.handshake_done:
                        if (time.time() - start_time) > 2.0:
                            dpg.set_value("state_handshake1", "WARNING:")
                            dpg.set_value("state_handshake2", "No \"speed set\" ACK from STM32")
                            self.handshake_done = True # Stop checking

                    # --- TRANSMISSION ---
                    try:
                        # Check if there is a message waiting to be sent
                        # block=False means it won't wait if the queue is empty
                        msg_to_send = self.tx_queue.get_nowait()
                        ser.write(msg_to_send)
                    except queue.Empty:
                        pass

                    # --- THE SMART CHECK ---
                    # If the STM32 hasn't sent anything, don't call read(1) yet: it allows the loop to go back up and check the tx_queue again
                    # The tiny sleep tells Windows: "I'm idle, let other programs work for 1ms" and keeps the CPU usage at 0%
                    if ser.in_waiting == 0:
                        time.sleep(0.001) # Breathe for 1ms, during this time the incoming bytes are bufferized by the hardware
                        continue          # (and USB driver) so nothing will be lost - the bucket usually holds 4,096 bytes or more
                                          # Also slows down the loop and prevent it to run millions of times per second
                                  
                    # No need for "if ser.in_waiting > 0:" because read() already waits for bytes                   
                    # --- RECEPTION (COBS Style) ---                 
                    data_in = ser.read(ser.in_waiting)    # Read all currently available bytes
                    if data_in:
                         # Iterates through data_in and adds each byte 1 by 1 to the end of the bytearray buffer
                        buffer.extend(data_in) # not mandatory (USB packets arrive always in one piece)
                        # The buffer could technically grow if "garbage" that never contains a 0x00 is received                   
                        if len(buffer) > 4096 and b'\x00' not in buffer: # Only checks the length if no delimiter is found to process
                            buffer.clear()                               # 4096 is a good size—large enough for any CAN burst,
                            self.err_frames += 1                         # but small enough to not impact RAM
                        # Look for the 0x00 delimiter
                        while b'\x00' in buffer:
                            # Extract the packet up to the first 0x00
                            packet_chunk, buffer = buffer.split(b'\x00', 1)

                            if len(packet_chunk) == 0:
                                continue # Ignore empty chunks from leading/double zeros

                            try:                                
                                decoded_msg = cobs.decode(packet_chunk)  # Decode COBS     

                                # process ACK (CAN speed - handshake) packet: [0x01 (LEN), 0xCF (CMD), 0x01 (STATUS)]
                                if not self.handshake_done:
                                    # Look specifically for the ACK: Len=0x01, Cmd=0xCF, Status=0x01
                                    if len(decoded_msg) == 3 and decoded_msg[1] == 0xCF:
                                        if decoded_msg[2] == 0x01:                                           
                                            self.handshake_done = True                                            
                                            dpg.set_value("state_handshake1", "Handshake Received Ok")
                                            continue # Move to next packet
                                        
                                 # process ACK (CAN speed - handshake) packet: [0x01 (LEN), 0xCF (CMD), 0x01 (STATUS)]
                                if len(decoded_msg) == 3 and decoded_msg[1] == 0xAF:
                                    if decoded_msg[2] == 0x01:                                     
                                        #dpg.set_value("filter_acked", "Filter settings applied !")
                                        self.speed_ack_blinking()
                                        continue # Move to next packet

                                # --- SAFETY CHECK 1: Length ---
                                # Minimum size: ID(2) + RTR(1) + DLC(1) = 4 bytes
                                if len(decoded_msg) < 4:
                                    self.err_frames += 1
                                    continue                                
                                                               
                                # Process the decoded_msg
                                # using int.from_bytes and simple list slicing (above), Python will handle this extremely quickly                              
                                can_id = int.from_bytes(decoded_msg[0:2], byteorder='little') # StdId
                                rtr = decoded_msg[2] # RTR
                                dlc = decoded_msg[3] # DLC
                                payload = decoded_msg[4:4+dlc]  # data                             
                                #  safety check : check calculated actual length vs LDC                               
                                now = datetime.now()
                                timestamp = f"{now.hour}:{now.minute}:{now.second}.{now.microsecond // 1000}"
                                frame = (
                                    timestamp,
                                    f"0x{can_id:03X}",
                                    f"{rtr:02X}",
                                    f"{dlc:02X}",
                                    [f"{b:02X}" for b in payload]
                                )
                                self.rx_queue.put(frame)                           

                            except Exception as e:
                                print(f"COBS Decode Error: {e}")
                                self.err_frames += 1

        except Exception as e:
            error_str = str(e)
            if "22" in error_str:
                msg = f"Hardware Reset on port {self.selected_port} (Code 22)"
            elif "13" in error_str:
                msg = f"Access Denied to port {self.selected_port} (Code 13)"
            elif " 2)" in error_str:
                msg = f"Device Not Found on port {self.selected_port} (Code 2)"
            else:
                msg = "General Serial Error"                        

            self.running = False            
            dpg.set_value("error_message_text", msg)
            dpg.configure_item("error_modal", show=True)
            dpg.configure_item("start_button_tag", label="Stopped")
            dpg.bind_item_theme("start_button_tag", self.t_off)
            dpg.configure_item("cercle", fill=[180, 15, 0])
            dpg.configure_item("port_dropdown", enabled=True)  # Re-enable the dropdown for COM ports         
            dpg.configure_item("can_speed_dropdown", enabled=True)  # Re-enable the dropdown for CAN speed
            dpg.bind_item_theme("error_modal", self.error_theme) 
            dpg.set_value("state_handshake1", "")
            dpg.set_value("state_handshake2", "")                   

    def send_command(self, sender, app_data):
        # Example: Sending a random command     
        command = b'\x02\x03\x04\x05\x0A' # Wrap your data in a bytes object
        self.tx_queue.put(command)

    def process_queue(self):
        """Process received frames in main thread (faster, GUI-safe)"""
        processed = 0
        max_per_frame = 50
        # 1. Get the current rows BEFORE starting the loop
        # This ensures 'rows' is defined and we know where the "top" is
        children = dpg.get_item_children("can_table_disp", 1)
        rows = children if children is not None else []

        while not self.rx_queue.empty() and processed < max_per_frame:
            frame = self.rx_queue.get()
            time_str, can_id, can_rtr, can_len, data_bytes = frame
            # Use the first row from our initial 'rows' list as the anchor
            # If the table is empty, rows[0] doesn't exist, so we use 0 (append)
            anchor = rows[0] if rows else 0

            # 'before' inserts the new row at the top of the table
            with dpg.table_row(parent="can_table_disp", before=anchor):
                dpg.add_text(time_str, color=(217, 252, 227))
                dpg.add_text(can_id, color=(242, 255, 142))
                dpg.add_text(" " + can_rtr)
                dpg.add_text(" " + can_len, color=(182, 211, 252))
                dpg.add_text("")
                for byte in data_bytes:
                    dpg.add_text(byte, color=(252, 200, 200))

            processed += 1
            self.nb_frames += 1

        # 3. Cleanup: Delete excess rows from the BOTTOM (oldest)
        # Re-fetch the current state after the loop to see how many we added
        current_children = dpg.get_item_children("can_table_disp", 1)
        if current_children:
            # If we exceed max_rows, delete the oldest ones at the end of the list
            while len(current_children) > self.max_rows:
                oldest_row = current_children.pop(-1) # Get the last item
                dpg.delete_item(oldest_row)

        dpg.set_value("nb_frames", self.nb_frames)
        dpg.set_value("error_frames", self.err_frames)

    def stop_all(self):       
        """Signals everything to stop."""
        self.running = False
        dpg.stop_dearpygui()


if __name__ == "__main__":
    # Instantiate the main application class to start the CAN monitor GUI
    app = STM32Monitor()

    while dpg.is_dearpygui_running():
        app.process_queue()   # ← CRITICAL FIX
        dpg.render_dearpygui_frame()
    
    print("Cleaning up...")
    dpg.destroy_context()

