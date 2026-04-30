import dearpygui.dearpygui as dpg
import time
import threading  

def load_assets():
    """Loads fonts and returns them as a dictionary for easy access."""
    with dpg.font_registry():
        fonts = {
            "XL": dpg.add_font("C:/Windows/Fonts/arial.ttf", 50),
            "L": dpg.add_font("C:/Windows/Fonts/arial.ttf", 30),
            "M": dpg.add_font("C:/Windows/Fonts/arial.ttf", 20),
            "S": dpg.add_font("C:/Windows/Fonts/arial.ttf", 16),
            "XS": dpg.add_font("C:/Windows/Fonts/arial.ttf", 14)
        }
    return fonts

def create_button_themes():
    """
    Creates and returns three button themes for use in the GUI.

    Returns:
        tuple: (start_off, start_on, stop_theme) where each is a DearPyGui theme object.
            - start_off: Theme for the Start button when it is OFF.
            - start_on: Theme for the Start button when it is ON.
            - stop_theme: Theme for the Stop button.
    """
    # Start Button OFF
    with dpg.theme() as start_off:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, [40, 120, 200])
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, [24, 80, 160])
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)

    # Start Button ON
    with dpg.theme() as start_on:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, [100, 100, 200])
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, [24, 80, 160])
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)

    # Stop Button
    with dpg.theme() as stop_theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, [130, 93, 25])
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, [156, 60, 76])
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)
     
    return start_off, start_on, stop_theme

def theme_table():  
    with dpg.theme() as table_theme:
        with dpg.theme_component(dpg.mvTable):           
            dpg.add_theme_color(dpg.mvThemeCol_Text, (240, 245, 240))   # Change the color of all text inside this table 
    return table_theme

def theme_error():
    with dpg.theme() as theme_win_error:
        with dpg.theme_component(dpg.mvAll): # mvAll targets everything in this window                 
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg, (26, 12, 12), category=dpg.mvThemeCat_Core)  # mvThemeCol_PopupBg is the background color         
            dpg.add_theme_color(dpg.mvThemeCol_Border, (158, 43, 43), category=dpg.mvThemeCat_Core)  # Change the border color too
    return theme_win_error

def toggle_start_handler(sender, app_data, user_data):  
    monitor, circle_tag, dropdown_port, can_speed_dropdown  = user_data    # user_data=[self, "cercle", "port_dropdown"]
    """Logic to toggle button text and theme."""
    # Inside your monitor class
    def run_blinker():
        monitor.blink_state = False
        while monitor.running:
            monitor.blink_state = not monitor.blink_state
            color = [12, 110, 0] if monitor.blink_state else [18, 169, 0]
            dpg.configure_item(circle_tag, fill=color)
            time.sleep(0.8)

    if not monitor.running:    
        monitor.running = True # Flip the flag
        monitor.nb_frames = 0
        monitor.clear_table()
        monitor.nb_frames = 0 
        monitor.err_frames = 0          
        threading.Thread(
            target=run_blinker,             
            daemon=True      # daemon=True tells Python: "If the main window closes, kill this thread immediately."
        ).start()      
        dpg.configure_item(sender, label="Running...")   # Update UI
        dpg.bind_item_theme(sender, monitor.t_on)              
        monitor.start_worker()  # Start the work (in the main file)    
    else:
       # Logic for stopping if you want the Start button to act as a toggle
        monitor.running = False
        dpg.set_value("state_handshake1", "")
        dpg.set_value("state_handshake2", "") 
        dpg.configure_item(circle_tag, fill=[180, 15, 0]) # red circle
        dpg.configure_item(sender, label="Stopped")
        dpg.bind_item_theme(sender, monitor.t_off)
        dpg.configure_item(dropdown_port, enabled=True)  # Re-enable the dropdown for COM ports   
        dpg.configure_item(can_speed_dropdown, enabled=True)  # Re-enable the dropdown for COM ports   
