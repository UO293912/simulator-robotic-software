import re
import tkinter as tk
import tkinter.messagebox as messagebox
from tkinter.filedialog import askopenfilename, asksaveasfilename
import tkinter.ttk as ttk
import graphics.controller as controller
import files.files_reader as files
import subprocess
import robot_components.robots as robots

DARK_BLUE = "#006468"
BLUE = "#17a1a5"


class MainApplication(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.title("Simulador Software para Robots")
        self.geometry("1280x720")
        # self.iconbitmap('assets/simlogo.ico')

        self.menu_bar = MenuBar(self, self)

        self.tools_frame = tk.Frame(self, bg=DARK_BLUE)
        self.button_bar = ButtonBar(self.tools_frame, self, bg=DARK_BLUE)
        self.selector_bar = SelectorBar(self.tools_frame, self, bg=DARK_BLUE)

        self.vertical_pane = tk.PanedWindow(
            orient=tk.VERTICAL, sashpad=5, sashrelief="solid", bg=DARK_BLUE)

        # Área de contenido: panel izquierdo de info + canvas central + notebook derecho
        self.content_area = tk.Frame(self.vertical_pane, bg=DARK_BLUE)

        # Panel izquierdo de información del brazo 3D (oculto por defecto)
        self.left_info_panel = Arm3DInfoPanel(self.content_area, bg=DARK_BLUE)

        # PanedWindow central+derecho
        self.center_right_pane = tk.PanedWindow(
            self.content_area, orient=tk.HORIZONTAL,
            sashpad=5, sashrelief="solid", bg=BLUE)

        self.drawing_frame = DrawingFrame(self.center_right_pane, self, bg=BLUE)

        # Notebook derecho: pestaña CÓDIGO y pestaña 3D opcional
        self.right_notebook = ttk.Notebook(self.center_right_pane)
        self.editor_frame = EditorFrame(self.right_notebook, bg=BLUE)
        self.arm3d_control_panel = Arm3DControlPanel(
            self.right_notebook, self, bg=DARK_BLUE,
            highlightthickness=1, highlightbackground="black")
        self.right_notebook.add(self.editor_frame, text="  CÓDIGO  ")

        self.console_frame = ConsoleFrame(self.vertical_pane, self, bg=DARK_BLUE)

        self.identifier = None
        self.controller = controller.RobotsController(self)
        self.prepare_controller()
        self.keys_used = True
        self.file_manager = files.FileManager()

        self.config(menu=self.menu_bar)
        self.button_bar.pack(fill=tk.X, side="left")
        self.selector_bar.pack(fill=tk.X, side="right")
        # These keys are the accepted ones for move the application. If you need more keys, added them here
        self.move_WASD = {
            "W": False,
            "A": False,
            "S": False,
            "D": False,
            "w": False,
            "a": False,
            "s": False,
            "d": False
        }

        self.tools_frame.pack(fill=tk.X)
        self.vertical_pane.pack(fill="both", expand=True)

        # Layout del área de contenido con grid (permite show/hide del panel izquierdo)
        self.content_area.grid_columnconfigure(0, weight=0)
        self.content_area.grid_columnconfigure(1, weight=1)
        self.content_area.grid_rowconfigure(0, weight=1)
        self.left_info_panel.grid(row=0, column=0, sticky="nsew")
        self.left_info_panel.grid_remove()  # oculto inicialmente
        self.center_right_pane.grid(row=0, column=1, sticky="nsew")

        # Notebook: CÓDIGO a la izquierda, CONTROL MANUAL a la derecha
        self.center_right_pane.add(self.drawing_frame, stretch="first", width=900, minsize=100)
        self.center_right_pane.add(self.right_notebook, width=280, minsize=100)

        self.vertical_pane.add(self.content_area, stretch="first", minsize=100)
        self.vertical_pane.add(self.console_frame, stretch="never", height=200, minsize=100)

        self.bind("<KeyPress>", self.key_press)
        self.bind("<KeyRelease>", self.key_release)
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.challenge = 0

    def prepare_controller(self):
        self.__update_robot()  # call first so the robot_layer is created
        self.controller.configure_layer(self.drawing_frame.canvas,
                                        self.drawing_frame.hud_canvas)  # call second so the canvas are initialized
        # call third so the console is initialized
        self.controller.configure_console(self.console_frame.console)
        # call last so all the three elements that are configured before are initialized
        self.__update_track()
        # if not, the track raises exception

    def execute(self):
        self.drawing_frame.canvas.focus_force()
        self.controller.execute(self.selector_bar.gamification_option_selector.current())

    def stop(self):
        self.controller.stop()

    def toggle_pause(self):
        """Alterna entre pausado y ejecutando (RF4.2.2)."""
        self.controller.toggle_pause()

    def step_once(self):
        """Ejecuta una sentencia más y vuelve a pausar (RF4.2.3)."""
        self.controller.step_once()

    def step_back(self):
        """Retrocede una sentencia en la ejecución pausada."""
        self.controller.step_back()

    def editor_undo(self):
        self.editor_frame.text.edit_undo()

    def editor_redo(self):
        self.editor_frame.text.edit_redo()

    def open_file(self, event=None):
        self.editor_frame.text.delete("1.0", tk.END)
        file = askopenfilename(filetypes=[("Arduino sketch", ".ino")])
        if file != '':
            content = self.file_manager.open(file)
            for line in content:
                self.editor_frame.text.insert(tk.END, f"{line}")

    def save_file(self, event=None):
        content = self.editor_frame.text.get("1.0", tk.END)
        file = asksaveasfilename(defaultextension=".ino", filetypes=[
            ("Arduino sketch", ".ino")])
        if file != '':
            self.file_manager.save(file, content)

    def get_code(self):
        return self.editor_frame.text.get("1.0", tk.END)

    def open_pin_configuration(self, event=None):
        """
        Top level window to configure pins connected to the
        Arduino board
        """
        robot = self.selector_bar.robot_selector.current()
        if robot == 5:
            self.open_arm3d_configuration()
        else:
            PinConfigurationWindow(self, robot, self)

    def open_arm3d_configuration(self):
        """Abre la ventana modal de configuración del brazo 3D."""
        import graphics.layers as _layers
        if isinstance(self.controller.robot_layer, _layers.Arm3DLayer):
            Arm3DConfigurationWindow(self, self.controller.robot_layer.motor3d, self)

    def zoom_in(self):
        self.controller.zoom_in()

    def zoom_out(self):
        self.controller.zoom_out()

    def change_zoom_label(self, zoom_level):
        self.drawing_frame.change_zoom_label(zoom_level)

    def change_robot(self, event):
        self.controller.stop()
        self.__update_robot()
        self.__update_track()  # Needed to set the circuit of the layer
        self.__update_gamification_option()
        self.__update_editor_frame_text()
        self.console_frame.console.config(state=tk.NORMAL)
        self.console_frame.console.insert(tk.END, "Robot cambiado con éxito\n")
        self.console_frame.console.config(state=tk.DISABLED)
        self.controller.robot_layer.is_drawing = False
        self.drawing_frame.key_drawing.deselect()

    def __update_robot(self):
        robot = self.selector_bar.robot_selector.current()
        self.controller.change_robot(robot)
        self.controller.configure_layer(
            self.drawing_frame.canvas, self.drawing_frame.hud_canvas)

    def change_track(self, event):
        self.controller.stop()
        self.__update_track()

    def change_gamification_option(self, event):
        self.controller.record_results(False, self.challenge)
        self.challenge = self.selector_bar.gamification_option_selector.current()
        self.controller.stop()
        self.__update_editor_frame_text()
        self.__update_gamification_option()
        self.controller.robot_layer.is_drawing = False
        self.drawing_frame.key_drawing.deselect()
        self.controller.delete_elements()

    def __update_track(self):
        circuit = self.selector_bar.track_selector.current()
        robot = self.selector_bar.robot_selector.current()
        if robot in (0, 1, 2):  # Aquí hay que mejorar los tipos de robots
            self.controller.change_circuit(circuit)
        self.controller.configure_layer(
            self.drawing_frame.canvas, self.drawing_frame.hud_canvas)

    def __update_editor_frame_text(self):
        if self.controller.board:
            challenge = self.selector_bar.gamification_option_selector.current()
            code = self.controller.robot_layer.drawing.get_robot_challenge(challenge).get_initial_code()
            self.editor_frame.change_text(code)

    def __update_gamification_option(self):
        if self.controller.board:
            challenge = self.selector_bar.gamification_option_selector.current()
            self.console_frame.console.config(state=tk.NORMAL)
            self.console_frame.console.delete("1.0", "end")
            self.show_buttons_gamification(True)
            text = "Nuevo intento. Puntuación inicial: 10\n\n"
            if challenge == 0:
                self.console_frame.console.insert(tk.END,
                                                  self.controller.robot_layer.drawing.
                                                  get_robot_challenge(0).get_challenge())
            if challenge == 1:
                self.console_frame.console.insert(tk.END,
                                                  self.controller.robot_layer.drawing.
                                                  get_robot_challenge(1).get_challenge())
            if challenge == 2:
                self.console_frame.console.insert(tk.END,
                                                  self.controller.robot_layer.drawing.
                                                  get_robot_challenge(2).get_challenge())
            if challenge == 3:
                self.console_frame.console.insert(tk.END,
                                                  self.controller.robot_layer.drawing.
                                                  get_robot_challenge(3).get_challenge())
            if challenge == 4:
                self.console_frame.console.insert(tk.END,
                                                  self.controller.robot_layer.drawing.
                                                  get_robot_challenge(4).get_challenge())
            if challenge == 5:
                self.console_frame.console.insert(tk.END,
                                                  self.controller.robot_layer.drawing.
                                                  get_robot_challenge(5).get_challenge())
            if challenge == 6:
                self.console_frame.console.insert(tk.END,
                                                  self.controller.robot_layer.drawing.
                                                  get_robot_challenge(6).get_challenge())
            self.controller.consoleGamification.write_encrypted(text, challenge + 1)
            self.console_frame.console.config(state=tk.DISABLED)
            self.controller.configure_layer(
                self.drawing_frame.canvas, self.drawing_frame.hud_canvas)
            self.controller.robot_layer.drawing.initialize_points()
            self.controller.new = True

    def show_circuit_selector(self, showing):
        if showing:
            self.selector_bar.recover_circuit_selector()
        else:
            self.selector_bar.hide_circuit_selector()

    def show_gamification_option_selector(self, showing):
        if showing:
            self.selector_bar.recover_gamification_option_selector()
        else:
            self.selector_bar.hide_gamification_option_selector()

    def show_joystick(self, showing):
        if showing:
            self.drawing_frame.show_joystick()
        else:
            self.drawing_frame.hide_joystick()

    def show_button_keys_movement(self, showing):
        if showing:
            self.drawing_frame.show_button_keys_movement()
        else:
            self.drawing_frame.hide_button_keys_movement()

    def show_buttons_gamification(self, showing):
        if showing:
            self.drawing_frame.show_buttons_gamification()
        else:
            self.drawing_frame.hide_buttons_gamification()

    def show_arm3d_panel(self, showing):
        if showing:
            self._set_arm3d_control_tab_visible(True)
            # Mostrar panel izquierdo de información
            self.left_info_panel.grid()
            # Ocultar HUD clásico (la info va al panel izquierdo)
            self.drawing_frame.hide_hud()
            # Mostrar botones de vista de cámara
            self.drawing_frame.show_arm3d_camera_buttons()
            # Seleccionar pestaña CONTROL MANUAL en el notebook
            self.right_notebook.select(self.arm3d_control_panel)
            # Conectar el panel de info al HUD del brazo (con retardo para que la capa esté lista)
            self.after(150, self._connect_arm3d_info_panel)
        else:
            self._set_arm3d_control_tab_visible(False)
            # Ocultar panel izquierdo
            self.left_info_panel.grid_remove()
            # Restaurar HUD clásico
            self.drawing_frame.show_hud()
            # Ocultar botones de cámara
            self.drawing_frame.hide_arm3d_camera_buttons()
            # Seleccionar pestaña CÓDIGO
            self.right_notebook.select(self.editor_frame)

    def _set_arm3d_control_tab_visible(self, visible):
        """Añade o elimina la pestaña CONTROL MANUAL según el robot activo."""
        control_tab = str(self.arm3d_control_panel)
        is_visible = control_tab in self.right_notebook.tabs()
        if visible and not is_visible:
            self.right_notebook.add(
                self.arm3d_control_panel, text="  CONTROL MANUAL  ")
        elif not visible and is_visible:
            if self.right_notebook.select() == control_tab:
                self.right_notebook.select(self.editor_frame)
            self.right_notebook.forget(self.arm3d_control_panel)

    def _connect_arm3d_info_panel(self):
        """Conecta el panel de info izquierdo al HUD del brazo 3D cuando la capa está lista.
        También actualiza los límites de los sliders y sincroniza el estado de los toggles
        de trayectoria y rangos articulares con el scene 3D."""
        try:
            import graphics.layers as _layers
            if isinstance(self.controller.robot_layer, _layers.Arm3DLayer):
                self.controller.robot_layer.hud._info_panel = self.left_info_panel
                self.arm3d_control_panel.update_joint_limits()
                # Sincronizar toggles UI → escena (evita el desfase checkbox=True / show_trail=False)
                self.controller.toggle_arm3d_trail(
                    self.arm3d_control_panel._trail_var.get())
                self.controller.toggle_arm3d_joint_ranges(
                    self.arm3d_control_panel._joint_ranges_var.get())
        except Exception:
            pass

    def show_hud(self, showing):
        if showing:
            self.drawing_frame.show_hud()
        else:
            self.drawing_frame.hide_hud()

    def show_keys_movements(self, show):
        """
        Show or hide the checkbox Keys Movement from the Hub
        :param show: boolean
        :return: None
        """
        if show:
            self.drawing_frame.key_movement.pack()
        else:
            self.drawing_frame.key_movement.forget()

    def show_key_drawing(self, show):
        """
        Show or hide the checkbox Drawing from the Hub
        :param show: boolean
        :return: None
        """
        if show:
            self.drawing_frame.key_drawing.pack()
        else:
            self.drawing_frame.key_drawing.forget()

    def key_press(self, event):
        pressed_key = event.char
        if pressed_key in self.move_WASD:
            self.move_WASD[pressed_key] = True

    def key_release(self, event):
        pressed_key = event.char
        if pressed_key in self.move_WASD:
            self.move_WASD[pressed_key] = False

    def abort_after(self):
        if self.identifier is not None:
            self.after_cancel(self.identifier)
            self.identifier = None

    def console_filter(self):
        msg_filters = {}
        if self.console_frame.output.get() == 1:
            msg_filters['info'] = True
        else:
            msg_filters['info'] = False
        if self.console_frame.warning.get() == 1:
            msg_filters['warning'] = True
        else:
            msg_filters['warning'] = False
        if self.console_frame.error.get() == 1:
            msg_filters['error'] = True
        else:
            msg_filters['error'] = False
        self.controller.filter_console(msg_filters)

    def toggle_keys(self):
        self.keys_used = not self.keys_used

    def set_drawing(self):
        self.controller.robot_layer.is_drawing = not self.controller.robot_layer.is_drawing
        if self.controller.robot_layer.is_drawing:
            self.controller.robot_layer.hud.drawing = None

    def close(self):
        self.controller.exit()
        self.controller.record_results(False, self.challenge)
        self.stop()
        self.destroy()


class PinConfigurationWindow(tk.Toplevel):

    def __init__(self, parent, robot_option, application: MainApplication = None, *args, **kwargs):
        tk.Toplevel.__init__(self, parent, *args, **kwargs)
        self.focus_force()
        self.application = application
        self.data = {}

        frame_content = tk.Frame(self)
        frame_buttons = tk.Frame(self)
        self.robot_option = robot_option

        # Actuator
        self.lb_actuator = tk.Label(frame_content, text="Actuador lineal:")
        self.lb_pin_bt1 = tk.Label(
            frame_content, text="Pin final izquierdo:", underline=10)
        self.entry_pin_bt1 = tk.Entry(frame_content)
        self.lb_pin_bt2 = tk.Label(
            frame_content, text="Pin final derecho:", underline=10)
        self.entry_pin_bt2 = tk.Entry(frame_content)
        self.lb_pin_joystick = tk.Label(
            frame_content, text="Pin botón Joystick:", underline=10)
        self.entry_pin_joystick = tk.Entry(frame_content)
        self.lb_pin_aservo = tk.Label(
            frame_content, text="Pin Servo:", underline=4)
        self.entry_pin_aservo = tk.Entry(frame_content)
        self.lb_pin_joystick_x = tk.Label(
            frame_content, text="Pin Joystick (x):", underline=14)
        self.entry_pin_joystick_x = tk.Entry(frame_content)
        self.lb_pin_joystick_y = tk.Label(
            frame_content, text="Pin Joystick (y):", underline=14)
        self.entry_pin_joystick_y = tk.Entry(frame_content)

        self.lb_mobile = tk.Label(frame_content, text="Robot móvil")
        self.lb_pin_servo1 = tk.Label(
            frame_content, text="Pin servo izquierdo:", underline=10)
        self.entry_pin_se1 = tk.Entry(frame_content)
        self.lb_pin_servo2 = tk.Label(
            frame_content, text="Pin servo derecho:", underline=10)
        self.entry_pin_se2 = tk.Entry(frame_content)
        self.lb_pin_light2 = tk.Label(
            frame_content, text="Pin luz izquierda:", underline=9)
        self.entry_pin_l2 = tk.Entry(frame_content)
        self.lb_pin_light3 = tk.Label(
            frame_content, text="Pin luz derecha:", underline=10)
        self.entry_pin_l3 = tk.Entry(frame_content)
        self.lb_pin_light1 = tk.Label(
            frame_content, text="Pin luz mas izquierda:", underline=14)
        self.entry_pin_l1 = tk.Entry(frame_content)
        self.lb_pin_light4 = tk.Label(
            frame_content, text="Pin luz mas derecha:", underline=17)
        self.entry_pin_l4 = tk.Entry(frame_content)
        self.lb_pin_sound1 = tk.Label(
            frame_content, text="Pin trigger:", underline=4)
        self.entry_pin_so1 = tk.Entry(frame_content)
        self.lb_pin_sound2 = tk.Label(
            frame_content, text="Pin echo:", underline=4)
        self.entry_pin_so2 = tk.Entry(frame_content)

        self.lb_board = tk.Label(frame_content, text="Placa arduino")
        self.lb_arduinoBoardComponent = tk.Entry(frame_content)

        self.btn_accept = tk.Button(
            frame_buttons, text="Aceptar", command=self.commit_data, underline=0)
        self.btn_cancel = tk.Button(
            frame_buttons, text="Cancelar", command=self.destroy, underline=0)

        if robot_option == 0:
            self.show_for_mobile2()
        if robot_option == 1:
            self.show_for_mobile3()
        if robot_option == 2:
            self.show_for_mobile4()
        if robot_option == 3:
            self.show_for_actuator()
        if robot_option == 4:
            self.show_for_arduinoboard()

        self.btn_accept.grid(row=0, column=0, sticky="se", padx=(0, 10))
        self.btn_cancel.grid(row=0, column=1, sticky="se")

        frame_content.pack(padx=5, pady=5)
        frame_buttons.pack(anchor="se", padx=5, pady=5)

        x = (parent.winfo_x() + (parent.winfo_width() / 2)) - \
            (self.winfo_reqwidth() / 2)
        y = (parent.winfo_y() + (parent.winfo_height() / 2)) - \
            (self.winfo_reqheight() / 2)
        self.geometry("+%d+%d" % (x, y))
        self.resizable(False, False)

        self.bind("<Alt-a>", self.commit_data)
        self.bind("<Alt-c>", lambda event: self.destroy())

    def commit_data(self, event=None):
        pin_data = {}
        if 'servo_left' in self.data:
            value = self.entry_pin_se1.get()
            if self.data['servo_left'] != value:
                pin_data['servo_left'] = value
        if 'servo_right' in self.data:
            value = self.entry_pin_se2.get()
            if self.data['servo_right'] != value:
                pin_data['servo_right'] = value
        if 'light_mleft' in self.data:
            value = self.entry_pin_l1.get()
            if self.data['light_mleft'] != value:
                pin_data['light_mleft'] = value
        if 'light_left' in self.data:
            value = self.entry_pin_l2.get()
            if self.data['light_left'] != value:
                pin_data['light_left'] = value
        if 'light_right' in self.data:
            value = self.entry_pin_l3.get()
            if self.data['light_right'] != value:
                pin_data['light_right'] = value
        if 'light_mright' in self.data:
            value = self.entry_pin_l4.get()
            if self.data['light_mright'] != value:
                pin_data['light_mright'] = value
        if 'sound_trig' in self.data:
            value = self.entry_pin_so1.get()
            if self.data['sound_trig'] != value:
                pin_data['sound_trig'] = value
        if 'sound_echo' in self.data:
            value = self.entry_pin_so2.get()
            if self.data['sound_echo'] != value:
                pin_data['sound_echo'] = value
        if 'button_left' in self.data:
            value = self.entry_pin_bt1.get()
            if self.data['button_left'] != value:
                pin_data['button_left'] = value
        if 'button_right' in self.data:
            value = self.entry_pin_bt2.get()
            if self.data['button_right'] != value:
                pin_data['button_right'] = value
        if 'servo' in self.data:
            value = self.entry_pin_aservo.get()
            if self.data['servo'] != value:
                pin_data['servo'] = value
        if 'button_joystick' in self.data:
            value = self.entry_pin_joystick.get()
            if self.data['button_joystick'] != value:
                pin_data['button_joystick'] = value
        if 'joystick_x' in self.data:
            value = self.entry_pin_joystick_x.get()
            if self.data['joystick_x'] != value:
                pin_data['joystick_x'] = value
        if 'joystick_y' in self.data:
            value = self.entry_pin_joystick_y.get()
            if self.data['joystick_y'] != value:
                pin_data['joystick_y'] = value
        self.application.controller.save_pin_data(pin_data)
        self.destroy()

    def show_for_mobile2(self):
        """
        Shows the window with the components needed to
        configure the mobile robot which has 2 light sensors
        """
        self.data = self.application.controller.get_pin_data()

        self.lb_mobile.grid(row=0, column=0, sticky="w")
        self.lb_pin_servo1.grid(row=1, column=0, sticky="w")
        self.entry_pin_se1.grid(row=1, column=1, sticky="w", padx=5)
        self.lb_pin_servo2.grid(row=1, column=2, sticky="w")
        self.entry_pin_se2.grid(row=1, column=3, sticky="w", padx=5)
        self.lb_pin_light2.grid(row=2, column=0, sticky="w")
        self.entry_pin_l2.grid(row=2, column=1, sticky="w", padx=5)
        self.lb_pin_light3.grid(row=2, column=2, sticky="w")
        self.entry_pin_l3.grid(row=2, column=3, sticky="w", padx=5)
        self.lb_pin_sound1.grid(row=4, column=0, sticky="w")
        self.entry_pin_so1.grid(row=4, column=1, sticky="w", padx=5)
        self.lb_pin_sound2.grid(row=4, column=2, sticky="w")
        self.entry_pin_so2.grid(row=4, column=3, sticky="w", padx=5)

        self.entry_pin_se1.insert(tk.END, self.data["servo_left"])
        self.entry_pin_se2.insert(tk.END, self.data["servo_right"])
        self.entry_pin_l2.insert(tk.END, self.data["light_left"])
        self.entry_pin_l3.insert(tk.END, self.data["light_right"])
        self.entry_pin_so1.insert(tk.END, self.data["sound_trig"])
        self.entry_pin_so2.insert(tk.END, self.data["sound_echo"])

        self.bind("<Alt-i>", lambda event: self.entry_pin_se1.focus())
        self.bind("<Alt-d>", lambda event: self.entry_pin_se2.focus())
        self.bind("<Alt-z>", lambda event: self.entry_pin_l2.focus())
        self.bind("<Alt-r>", lambda event: self.entry_pin_l3.focus())
        self.bind("<Alt-t>", lambda event: self.entry_pin_so1.focus())
        self.bind("<Alt-e>", lambda event: self.entry_pin_so2.focus())

    def show_for_mobile3(self):
        """
        Shows the window with the components needed to
        configure the mobile robot which has 3 light sensors
        """
        self.data = self.application.controller.get_pin_data()

        self.lb_mobile.grid(row=0, column=0, sticky="w")
        self.lb_pin_servo1.grid(row=1, column=0, sticky="w")
        self.entry_pin_se1.grid(row=1, column=1, sticky="w", padx=5)
        self.lb_pin_servo2.grid(row=1, column=2, sticky="w")
        self.entry_pin_se2.grid(row=1, column=3, sticky="w", padx=5)
        self.lb_pin_light2.grid(row=2, column=0, sticky="w")
        self.entry_pin_l2.grid(row=2, column=1, sticky="w", padx=5)
        self.lb_pin_light3.grid(row=2, column=2, sticky="w")
        self.entry_pin_l3.grid(row=2, column=3, sticky="w", padx=5)
        self.lb_pin_light1.grid(row=3, column=0, sticky="w")
        self.entry_pin_l1.grid(row=3, column=1, sticky="w", padx=5)
        # self.lb_pin_light4.grid(row=3, column=2, sticky="w")
        # self.entry_pin_l4.grid(row=3, column=3, sticky="w", padx=5)
        self.lb_pin_sound1.grid(row=4, column=0, sticky="w")
        self.entry_pin_so1.grid(row=4, column=1, sticky="w", padx=5)
        self.lb_pin_sound2.grid(row=4, column=2, sticky="w")
        self.entry_pin_so2.grid(row=4, column=3, sticky="w", padx=5)

        self.entry_pin_se1.insert(tk.END, self.data["servo_left"])
        self.entry_pin_se2.insert(tk.END, self.data["servo_right"])
        self.entry_pin_l1.insert(tk.END, self.data["light_mleft"])
        self.entry_pin_l2.insert(tk.END, self.data["light_left"])
        self.entry_pin_l3.insert(tk.END, self.data["light_right"])
        # self.entry_pin_l4.insert(tk.END, self.data["light_mright"])
        self.entry_pin_so1.insert(tk.END, self.data["sound_trig"])
        self.entry_pin_so2.insert(tk.END, self.data["sound_echo"])

        self.bind("<Alt-i>", lambda event: self.entry_pin_se1.focus())
        self.bind("<Alt-d>", lambda event: self.entry_pin_se2.focus())
        self.bind("<Alt-z>", lambda event: self.entry_pin_l2.focus())
        self.bind("<Alt-r>", lambda event: self.entry_pin_l3.focus())
        self.bind("<Alt-q>", lambda event: self.entry_pin_l1.focus())
        # self.bind("<Alt-h>", lambda event: self.entry_pin_l4.focus())
        self.bind("<Alt-t>", lambda event: self.entry_pin_so1.focus())
        self.bind("<Alt-e>", lambda event: self.entry_pin_so2.focus())

    def show_for_mobile4(self):
        """
        Shows the window with the components needed to
        configure the mobile robot which has 4 light sensors
        """
        self.data = self.application.controller.get_pin_data()

        self.lb_mobile.grid(row=0, column=0, sticky="w")
        self.lb_pin_servo1.grid(row=1, column=0, sticky="w")
        self.entry_pin_se1.grid(row=1, column=1, sticky="w", padx=5)
        self.lb_pin_servo2.grid(row=1, column=2, sticky="w")
        self.entry_pin_se2.grid(row=1, column=3, sticky="w", padx=5)
        self.lb_pin_light2.grid(row=2, column=0, sticky="w")
        self.entry_pin_l2.grid(row=2, column=1, sticky="w", padx=5)
        self.lb_pin_light3.grid(row=2, column=2, sticky="w")
        self.entry_pin_l3.grid(row=2, column=3, sticky="w", padx=5)
        self.lb_pin_light1.grid(row=3, column=0, sticky="w")
        self.entry_pin_l1.grid(row=3, column=1, sticky="w", padx=5)
        self.lb_pin_light4.grid(row=3, column=2, sticky="w")
        self.entry_pin_l4.grid(row=3, column=3, sticky="w", padx=5)
        self.lb_pin_sound1.grid(row=4, column=0, sticky="w")
        self.entry_pin_so1.grid(row=4, column=1, sticky="w", padx=5)
        self.lb_pin_sound2.grid(row=4, column=2, sticky="w")
        self.entry_pin_so2.grid(row=4, column=3, sticky="w", padx=5)

        self.entry_pin_se1.insert(tk.END, self.data["servo_left"])
        self.entry_pin_se2.insert(tk.END, self.data["servo_right"])
        self.entry_pin_l1.insert(tk.END, self.data["light_mleft"])
        self.entry_pin_l2.insert(tk.END, self.data["light_left"])
        self.entry_pin_l3.insert(tk.END, self.data["light_right"])
        self.entry_pin_l4.insert(tk.END, self.data["light_mright"])
        self.entry_pin_so1.insert(tk.END, self.data["sound_trig"])
        self.entry_pin_so2.insert(tk.END, self.data["sound_echo"])

        self.bind("<Alt-i>", lambda event: self.entry_pin_se1.focus())
        self.bind("<Alt-d>", lambda event: self.entry_pin_se2.focus())
        self.bind("<Alt-z>", lambda event: self.entry_pin_l2.focus())
        self.bind("<Alt-r>", lambda event: self.entry_pin_l3.focus())
        self.bind("<Alt-q>", lambda event: self.entry_pin_l1.focus())
        self.bind("<Alt-h>", lambda event: self.entry_pin_l4.focus())
        self.bind("<Alt-t>", lambda event: self.entry_pin_so1.focus())
        self.bind("<Alt-e>", lambda event: self.entry_pin_so2.focus())

    def show_for_actuator(self):
        """
        Shows the window with the components needed to
        configure the actuator
        """
        self.data = self.application.controller.get_pin_data()

        self.lb_actuator.grid(row=0, column=0, sticky="w")
        self.lb_pin_bt1.grid(row=1, column=0, sticky="w")
        self.entry_pin_bt1.grid(row=1, column=1, sticky="w", padx=5)
        self.lb_pin_bt2.grid(row=1, column=2, sticky="w")
        self.entry_pin_bt2.grid(row=1, column=3, sticky="w", padx=5)
        self.lb_pin_joystick.grid(row=2, column=0, sticky="w")
        self.entry_pin_joystick.grid(row=2, column=1, sticky="w", padx=5)
        self.lb_pin_aservo.grid(row=2, column=2, sticky="w")
        self.entry_pin_aservo.grid(row=2, column=3, sticky="w", padx=5)
        self.lb_pin_joystick_x.grid(row=3, column=0, sticky="w")
        self.entry_pin_joystick_x.grid(row=3, column=1, sticky="w", padx=5)
        self.lb_pin_joystick_y.grid(row=3, column=2, sticky="w")
        self.entry_pin_joystick_y.grid(row=3, column=3, sticky="w", padx=5)

        self.entry_pin_bt1.insert(tk.END, self.data["button_left"])
        self.entry_pin_bt2.insert(tk.END, self.data["button_right"])
        self.entry_pin_joystick.insert(tk.END, self.data["button_joystick"])
        self.entry_pin_aservo.insert(tk.END, self.data["servo"])
        self.entry_pin_joystick_x.insert(tk.END, self.data["joystick_x"])
        self.entry_pin_joystick_y.insert(tk.END, self.data["joystick_y"])

        self.bind("<Alt-i>", lambda event: self.entry_pin_bt1.focus())
        self.bind("<Alt-d>", lambda event: self.entry_pin_bt2.focus())
        self.bind("<Alt-j>", lambda event: self.entry_pin_joystick.focus())
        self.bind("<Alt-s>", lambda event: self.entry_pin_aservo.focus())
        self.bind("<Alt-x>", lambda event: self.entry_pin_joystick_x.focus())
        self.bind("<Alt-y>", lambda event: self.entry_pin_joystick_y.focus())

    def show_for_arduinoboard(self):
        """
        Shows the window with the components needed to
        configure the arduino board which has an arduino board
        """
        self.data = self.application.controller.get_pin_data()

        self.lb_board.grid(row=0, column=0, sticky="w")
        self.lb_arduinoBoardComponent.grid(row=1, column=0, sticky="w")


class Arm3DConfigurationWindow(tk.Toplevel):
    """
    Ventana modal de configuración del brazo 3D.
    Permite modificar DOF, tabla DH, tipos de articulación y límites.
    Importar/exportar configuración en JSON.
    """

    def __init__(self, parent, motor3d_api, application: MainApplication = None, *args, **kwargs):
        tk.Toplevel.__init__(self, parent, *args, **kwargs)
        self.title("Configuración Brazo 3D")
        self.focus_force()
        self.resizable(True, True)
        self.application = application
        self.motor3d = motor3d_api

        # ---- Frame superior: perfil y DOF ----
        top_frame = tk.Frame(self, bg=DARK_BLUE)
        top_frame.pack(fill=tk.X, padx=8, pady=6)

        tk.Label(top_frame, text="Perfil:", bg=DARK_BLUE, fg="white",
                 font=("Consolas", 12)).pack(side=tk.LEFT)
        self._presets = self.motor3d.repository.list_builtin_presets()
        preset_names = ["Custom"] + sorted(self._presets.keys())
        self._preset_var = tk.StringVar(value="Custom")
        preset_combo = ttk.Combobox(top_frame, textvariable=self._preset_var,
                                    values=preset_names, state="readonly", width=18)
        preset_combo.pack(side=tk.LEFT, padx=(4, 16))
        preset_combo.bind("<<ComboboxSelected>>", self._on_preset_selected)

        tk.Label(top_frame, text="DOF:", bg=DARK_BLUE, fg="white",
                 font=("Consolas", 12)).pack(side=tk.LEFT)
        self._dof_var = tk.IntVar(value=self.motor3d.model.dof)
        dof_spin = tk.Spinbox(top_frame, from_=1, to=6, textvariable=self._dof_var,
                              width=4, font=("Consolas", 12),
                              command=self._on_dof_change)
        dof_spin.pack(side=tk.LEFT, padx=(4, 20))

        tk.Label(top_frame, text="Modo visual:", bg=DARK_BLUE, fg="white",
                 font=("Consolas", 12)).pack(side=tk.LEFT)
        self._visual_var = tk.StringVar()
        modes = ["auto_generic", "braccio_exact", "skeleton"]
        current_mode = self.motor3d.model.visual.get('mode', 'auto_generic')
        self._visual_var.set(current_mode)
        vis_combo = ttk.Combobox(top_frame, textvariable=self._visual_var,
                                 values=modes, state="readonly", width=15)
        vis_combo.pack(side=tk.LEFT, padx=(4, 0))

        # ---- Tabla DH ----
        table_frame = tk.Frame(self)
        table_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        headers = ["J", "Theta (°)", "d (mm)", "a (mm)", "Alpha (°)", "Tipo", "Lím min", "Lím max"]
        for col, h in enumerate(headers):
            tk.Label(table_frame, text=h, font=("Consolas", 11, "bold"),
                     relief=tk.RIDGE, padx=4).grid(row=0, column=col, sticky="nsew")

        self._rows = []
        self._build_dh_rows(table_frame)

        self._table_frame = table_frame

        # ---- Botones inferiores ----
        btn_frame = tk.Frame(self, bg=DARK_BLUE)
        btn_frame.pack(fill=tk.X, padx=8, pady=6)

        tk.Button(btn_frame, text="Import JSON", bg=BLUE, fg=DARK_BLUE,
                  font=("Consolas", 11), command=self._import_json).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_frame, text="Export Config", bg=BLUE, fg=DARK_BLUE,
                  font=("Consolas", 11), command=self._export_json).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_frame, text="Cancel", bg=DARK_BLUE, fg="white",
                  font=("Consolas", 11), command=self.destroy).pack(side=tk.RIGHT, padx=4)
        tk.Button(btn_frame, text="SAVE", bg="#00AA55", fg="white",
                  font=("Consolas", 11, "bold"), command=self._save).pack(side=tk.RIGHT, padx=4)

        # Centrar ventana
        self.update_idletasks()
        x = parent.winfo_x() + (parent.winfo_width() - self.winfo_reqwidth()) // 2
        y = parent.winfo_y() + (parent.winfo_height() - self.winfo_reqheight()) // 2
        self.geometry("+%d+%d" % (x, y))

    def _build_dh_rows(self, table_frame):
        """Construye las filas editables de la tabla DH."""
        # Limpiar filas anteriores
        for widgets in self._rows:
            for w in widgets:
                try:
                    w.destroy()
                except Exception:
                    pass
        self._rows = []

        model = self.motor3d.model
        n = self._dof_var.get()

        for i in range(n):
            row_entries = []
            dh = model.dh_rows[i] if i < len(model.dh_rows) else {'theta': 0, 'd': 0, 'a': 100, 'alpha': 0}
            lims = model.joint_limits[i] if i < len(model.joint_limits) else (-90.0, 90.0)
            jtype = model.joint_types[i] if i < len(model.joint_types) else 'R'

            tk.Label(table_frame, text=str(i + 1), font=("Consolas", 11),
                     width=3).grid(row=i + 1, column=0, sticky="nsew", padx=1, pady=1)

            for col, key in enumerate(['theta', 'd', 'a', 'alpha']):
                e = tk.Entry(table_frame, font=("Consolas", 11), width=9)
                e.insert(0, str(round(float(dh.get(key, 0.0)), 2)))
                e.grid(row=i + 1, column=col + 1, sticky="nsew", padx=1, pady=1)
                row_entries.append(e)

            # Tipo articulación
            type_var = tk.StringVar(value=jtype)
            type_combo = ttk.Combobox(table_frame, textvariable=type_var,
                                      values=["R", "P"], state="readonly", width=4)
            type_combo.grid(row=i + 1, column=5, sticky="nsew", padx=1, pady=1)
            row_entries.append(type_combo)

            # Límites
            e_min = tk.Entry(table_frame, font=("Consolas", 11), width=8)
            e_min.insert(0, str(round(float(lims[0]), 1)))
            e_min.grid(row=i + 1, column=6, sticky="nsew", padx=1, pady=1)

            e_max = tk.Entry(table_frame, font=("Consolas", 11), width=8)
            e_max.insert(0, str(round(float(lims[1]), 1)))
            e_max.grid(row=i + 1, column=7, sticky="nsew", padx=1, pady=1)

            row_entries.extend([e_min, e_max])
            self._rows.append(row_entries)

    def _on_preset_selected(self, _event=None):
        """Carga el preset seleccionado y repuebla DOF, modo visual y tabla DH."""
        name = self._preset_var.get()
        if name == "Custom":
            return
        path = self._presets.get(name)
        if not path:
            return
        import json
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception:
            tk.messagebox.showerror("Error", f"No se pudo leer el preset '{name}'.", parent=self)
            return
        dof = data.get('dof', self.motor3d.model.dof)
        self._dof_var.set(dof)
        # Actualizar temporalmente el modelo con los datos del preset para que
        # _build_dh_rows pueda leerlos
        self.motor3d.set_model_config(data)
        visual_mode = data.get('visual', {}).get('mode', 'auto_generic')
        self._visual_var.set(visual_mode)
        self._build_dh_rows(self._table_frame)

    def _on_dof_change(self):
        self._build_dh_rows(self._table_frame)

    def _collect_config(self):
        """Lee los valores de la tabla y retorna (dict, None) o (None, mensaje_error).
        Resalta en rojo las entradas inválidas para dar feedback visual al usuario."""
        n = len(self._rows)
        dh_rows = []
        joint_types = []
        joint_limits = []
        errors = []

        # Limpiar resaltados previos
        for row in self._rows:
            for widget in (row[0], row[1], row[2], row[3], row[5], row[6]):
                try:
                    widget.configure(highlightthickness=0)
                except Exception:
                    pass

        def _mark_invalid(widget, msg):
            try:
                widget.configure(highlightthickness=2,
                                 highlightbackground="#FF4444",
                                 highlightcolor="#FF4444")
            except Exception:
                pass
            errors.append(msg)

        for i, row in enumerate(self._rows):
            joint_n = i + 1
            dh_ok = True
            parsed = {}
            for col, key in enumerate(['theta', 'd', 'a', 'alpha']):
                try:
                    parsed[key] = float(row[col].get())
                except ValueError:
                    _mark_invalid(row[col], f"J{joint_n}: '{key}' no es un número válido.")
                    dh_ok = False

            if dh_ok:
                # RF1.1.1.3: longitud de eslabón 'a' ∈ [0, 2000] mm
                a = parsed['a']
                if a < 0:
                    _mark_invalid(row[2], f"J{joint_n}: 'a' debe ser ≥ 0 mm.")
                    dh_ok = False
                elif a > 2000:
                    _mark_invalid(row[2], f"J{joint_n}: 'a' supera el límite físico (2000 mm).")
                    dh_ok = False

            if dh_ok:
                dh_rows.append({'theta': parsed['theta'], 'd': parsed['d'],
                                'a': parsed['a'], 'alpha': parsed['alpha']})
            else:
                dh_rows.append({'theta': 0, 'd': 0, 'a': 100, 'alpha': 0})  # placeholder

            jtype = row[4].get() if hasattr(row[4], 'get') else 'R'
            joint_types.append(jtype)

            lim_ok = True
            try:
                lim_min = float(row[5].get())
            except ValueError:
                _mark_invalid(row[5], f"J{joint_n}: límite mínimo no es un número válido.")
                lim_ok = False
                lim_min = -90.0

            try:
                lim_max = float(row[6].get())
            except ValueError:
                _mark_invalid(row[6], f"J{joint_n}: límite máximo no es un número válido.")
                lim_ok = False
                lim_max = 90.0

            if lim_ok and lim_min >= lim_max:
                _mark_invalid(row[5], f"J{joint_n}: límite mínimo ≥ máximo.")
                _mark_invalid(row[6], f"J{joint_n}: límite mínimo ≥ máximo.")

            joint_limits.append((lim_min, lim_max))

        if errors:
            return None, "\n".join(errors)

        current = self.motor3d.get_model_config()
        current['dof'] = n
        current['dh_rows'] = dh_rows
        current['joint_types'] = joint_types
        current['joint_limits'] = joint_limits
        current['visual'] = dict(current.get('visual', {}))
        current['visual']['mode'] = self._visual_var.get()

        return current, None

    def _save(self):
        config, error = self._collect_config()
        if error:
            tk.messagebox.showerror("Error de configuración", error, parent=self)
            return
        self.motor3d.set_model_config(config)
        self.motor3d.save_model_config()
        self.destroy()

    def _import_json(self):
        from tkinter.filedialog import askopenfilename
        path = askopenfilename(filetypes=[("JSON config", "*.json")], parent=self)
        if path:
            ok = self.motor3d.load_model_config(path=path)
            if ok:
                self._dof_var.set(self.motor3d.model.dof)
                self._build_dh_rows(self._table_frame)
            else:
                tk.messagebox.showerror("Error", "No se pudo cargar el archivo JSON.", parent=self)

    def _export_json(self):
        from tkinter.filedialog import asksaveasfilename
        path = asksaveasfilename(defaultextension=".json",
                                 filetypes=[("JSON config", "*.json")], parent=self)
        if path:
            config, error = self._collect_config()
            if error:
                tk.messagebox.showerror("Error de configuración", error, parent=self)
                return
            self.motor3d.set_model_config(config)
            ok = self.motor3d.save_model_config(path=path)
            if not ok:
                tk.messagebox.showerror("Error", "No se pudo guardar el archivo.", parent=self)


class MenuBar(tk.Menu):

    def __init__(self, parent, application: MainApplication = None, *args, **kwargs):
        tk.Menu.__init__(self, parent, *args, **kwargs)
        self.application = application

        file_menu = tk.Menu(self, tearoff=0)
        file_menu.add_command(label="Nuevo archivo",
                              command=self.create_file, accelerator="Ctrl+N")
        file_menu.add_separator()
        file_menu.add_command(
            label="Importar sketch", command=application.open_file, accelerator="Ctrl+O")
        file_menu.add_command(
            label="Guardar sketch", command=application.save_file, accelerator="Ctrl+S")
        file_menu.add_separator()
        file_menu.add_command(
            label="Salir", command=self.check_if_exit, accelerator="Alt+F4")
        self.add_cascade(label="Archivo", menu=file_menu)

        edit_menu = tk.Menu(self, tearoff=0)
        edit_menu.add_command(
            label="Deshacer", command=application.editor_undo, accelerator="Ctrl+Z")
        edit_menu.add_command(
            label="Rehacer", command=application.editor_redo, accelerator="Ctrl+Y")
        self.add_cascade(label="Editar", menu=edit_menu)

        conf_menu = tk.Menu(self, tearoff=0)
        conf_menu.add_command(label="Configurar pines", command=application.open_pin_configuration,
                              accelerator="Ctrl+P")
        self.add_cascade(label="Configurar", menu=conf_menu)

        exec_menu = tk.Menu(self, tearoff=0)
        exec_menu.add_command(
            label="Ejecutar", command=application.execute, accelerator="F5")
        exec_menu.add_command(
            label="Detener", command=application.stop, accelerator="Ctrl+F5")
        exec_menu.add_separator()
        exec_menu.add_command(
            label="Pausar / Reanudar", command=application.toggle_pause, accelerator="F8")
        exec_menu.add_command(
            label="Paso a paso", command=application.step_once, accelerator="F10")
        exec_menu.add_separator()
        exec_menu.add_command(
            label="Ampliar", command=lambda event: application.zoom_in(), accelerator="Ctrl++")
        exec_menu.add_command(
            label="Reducir", command=lambda event: application.zoom_out(), accelerator="Ctrl+-")
        self.add_cascade(label="Ejecutar", menu=exec_menu)

        help_menu = tk.Menu(self, tearoff=0)
        help_menu.add_command(label="Manual de ayuda",
                              command=self.__launch_help, accelerator="Ctrl+H")
        help_menu.add_command(
            label="Acerca de", command=self.show_about, accelerator="Ctrl+A")
        self.add_cascade(label="Ayuda", menu=help_menu)

        self.bind_all("<Control-p>", application.open_pin_configuration)
        self.bind_all("<Control-n>", self.create_file)
        self.bind_all("<Control-o>", application.open_file)
        self.bind_all("<Control-s>", application.save_file)
        self.bind_all("<Control-h>", self.__launch_help)
        self.bind_all("<Control-a>", self.show_about)
        self.bind_all("<F5>", application.execute)
        self.bind_all("<Control-F5>", application.stop)
        self.bind_all("<F8>", lambda e: application.toggle_pause())
        self.bind_all("<F10>", lambda e: application.step_once())
        self.bind_all("<Control-plus>", lambda event: application.zoom_in())
        self.bind_all("<Control-minus>", lambda event: application.zoom_out())

    def __launch_help(self, event=None):
        subprocess.Popen('manual-usuario.pdf', shell=True)

    def create_file(self, event=None):
        if messagebox.askyesno('Nuevo archivo',
                               '¿Seguro que quieres crear un nuevo archivo? Se perderá el sketch si no está guardado'):
            self.application.editor_frame.create_file()

    def check_if_exit(self):
        if messagebox.askyesno('Salir', '¿Seguro que quieres salir? Se perderá el sketch si no está guardado'):
            self.application.close()

    def show_about(self, event=None):
        messagebox.showinfo('Simulador de Software para robots',
                            str(
                                'Aplicación realizada como trabajo de fin de grado.\n'
                                + 'Autor inicial: Diego Fernández Suárez\n'
                                + 'Autora extensión: María Suárez Hevia\n'
                                + 'Autor extensión 3D (Braccio): Nicolás Guerbartchouk Pérez\n'
                                + 'Tutor: Cristian González García\n'
                                + 'Versión actual: b-0.6'
                            ))


class Arm3DControlPanel(tk.Frame):
    """Panel de control manual del brazo robótico 3D — layout vertical para pestaña derecha.
    Muestra sliders J1-J6 (cinemática directa) y campos IK (X/Y/Z).
    Visible únicamente cuando el robot Braccio (opción 5) está seleccionado.
    """

    _JOINT_LABELS = ["J1 Base", "J2 Hombro", "J3 Codo", "J4 Muñeca V", "J5 Muñeca R", "J6 Pinza"]
    _JOINT_LIMITS = [(0, 180), (15, 165), (0, 180), (0, 180), (0, 180), (10, 73)]

    def __init__(self, parent, application: 'MainApplication' = None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.application = application

        # --- Cinemática Directa: sliders verticales ---
        tk.Label(self, text="Cinemática Directa",
                 bg=DARK_BLUE, fg="#00E5CC",
                 font=("Consolas", 10, "bold")).pack(anchor="w", padx=8, pady=(8, 2))

        sliders_container = tk.Frame(self, bg=DARK_BLUE)
        sliders_container.pack(fill=tk.X, padx=4)
        sliders_container.columnconfigure(1, weight=1)

        self._sliders = []
        self._val_labels = []
        for i in range(6):
            lim_min, lim_max = self._JOINT_LIMITS[i]

            tk.Label(sliders_container, text=self._JOINT_LABELS[i],
                     bg=DARK_BLUE, fg="white", font=("Consolas", 9),
                     width=12, anchor="e").grid(row=i, column=0, padx=(4, 2), pady=1, sticky="e")

            slider = tk.Scale(
                sliders_container,
                from_=lim_min, to=lim_max,
                orient=tk.HORIZONTAL,
                bg=DARK_BLUE, fg="white",
                sliderrelief=tk.FLAT, highlightthickness=0,
                width=12, showvalue=False,
                command=lambda val, idx=i: self._on_slider(idx, val)
            )
            slider.set(90)
            slider.grid(row=i, column=1, padx=2, pady=1, sticky="ew")

            val_lbl = tk.Label(sliders_container, text=" 90°",
                               bg=DARK_BLUE, fg="white",
                               font=("Consolas", 9), width=5, anchor="w")
            val_lbl.grid(row=i, column=2, padx=(2, 4), pady=1, sticky="w")

            self._sliders.append(slider)
            self._val_labels.append(val_lbl)

        # --- Separador ---
        tk.Frame(self, bg=BLUE, height=1).pack(fill=tk.X, padx=6, pady=6)

        # --- Cinemática Inversa ---
        tk.Label(self, text="Cinemática Inversa",
                 bg=DARK_BLUE, fg="#00E5CC",
                 font=("Consolas", 10, "bold")).pack(anchor="w", padx=8, pady=(0, 4))

        ik_coords = tk.Frame(self, bg=DARK_BLUE)
        ik_coords.pack(fill=tk.X, padx=8)

        for col_i, axis in enumerate(["X", "Y", "Z"]):
            tk.Label(ik_coords, text=f"{axis}:", bg=DARK_BLUE, fg="white",
                     font=("Consolas", 10)).grid(row=0, column=col_i * 2, padx=(4, 1), sticky="e")
        self._entry_x = tk.Entry(ik_coords, width=6, font=("Consolas", 10))
        self._entry_y = tk.Entry(ik_coords, width=6, font=("Consolas", 10))
        self._entry_z = tk.Entry(ik_coords, width=6, font=("Consolas", 10))
        self._entry_x.insert(0, "0")
        self._entry_y.insert(0, "0")
        self._entry_z.insert(0, "400")
        self._entry_x.grid(row=0, column=1, padx=(0, 4))
        self._entry_y.grid(row=0, column=3, padx=(0, 4))
        self._entry_z.grid(row=0, column=5, padx=(0, 4))

        btn_frame = tk.Frame(self, bg=DARK_BLUE)
        btn_frame.pack(fill=tk.X, padx=8, pady=4)

        self._btn_ik = tk.Button(btn_frame, text="Confirmar Posición",
                                 bg=BLUE, bd=0, fg=DARK_BLUE,
                                 font=("Consolas", 10), command=self._on_ik)
        self._btn_ik.pack(fill=tk.X, pady=(0, 3))

        self._btn_reset_cam = tk.Button(btn_frame, text="Reset Cámara",
                                        bg=BLUE, bd=0, fg=DARK_BLUE,
                                        font=("Consolas", 10), command=self._on_reset_cam)
        self._btn_reset_cam.pack(fill=tk.X)

        self._lbl_ik_status = tk.Label(self, text="", bg=DARK_BLUE, fg="#aaffaa",
                                       font=("Consolas", 9), wraplength=240, justify=tk.LEFT)
        self._lbl_ik_status.pack(anchor="w", padx=8, pady=2)

        # --- Separador ---
        tk.Frame(self, bg=BLUE, height=1).pack(fill=tk.X, padx=6, pady=4)

        # --- Toggles de visualización ---
        toggles_frame = tk.Frame(self, bg=DARK_BLUE)
        toggles_frame.pack(fill=tk.X, padx=8)

        self._trail_var = tk.BooleanVar(value=True)
        tk.Checkbutton(toggles_frame, text="Trayectoria",
                       variable=self._trail_var,
                       bg=DARK_BLUE, fg="white", selectcolor=DARK_BLUE,
                       font=("Consolas", 9), activebackground=DARK_BLUE,
                       command=self._on_trail_toggle).pack(side=tk.LEFT, padx=(0, 8))

        self._joint_ranges_var = tk.BooleanVar(value=False)
        tk.Checkbutton(toggles_frame, text="Rangos articulares",
                       variable=self._joint_ranges_var,
                       bg=DARK_BLUE, fg="white", selectcolor=DARK_BLUE,
                       font=("Consolas", 9), activebackground=DARK_BLUE,
                       command=self._on_joint_ranges_toggle).pack(side=tk.LEFT)

    # ------------------------------------------------------------------ handlers

    def _on_slider(self, joint_idx, val):
        try:
            float_val = float(val)
            self._val_labels[joint_idx].config(text=f"{int(float_val):>3}°")
            self.application.controller.update_arm3d_joint(joint_idx, float_val - 90.0)
        except Exception:
            pass

    def _on_ik(self):
        # Resetear borde de los entries
        for entry in (self._entry_x, self._entry_y, self._entry_z):
            entry.configure(highlightthickness=0)
        invalid = False
        parsed = []
        for entry in (self._entry_x, self._entry_y, self._entry_z):
            try:
                parsed.append(float(entry.get()))
            except ValueError:
                entry.configure(highlightthickness=2, highlightbackground="#FF4444",
                                highlightcolor="#FF4444")
                invalid = True
        if invalid:
            self._lbl_ik_status.config(text="Valores X/Y/Z inválidos", fg="#ff8888")
            return
        x, y, z = parsed
        converged, msg = self.application.controller.solve_arm3d_ik(x, y, z)
        if converged:
            self._lbl_ik_status.config(text=str(msg), fg="#aaffaa")
            self._sync_sliders_from_model()
        else:
            self._lbl_ik_status.config(text=str(msg), fg="#ffcc88")

    def _on_reset_cam(self):
        self.application.controller.reset_arm3d_camera()

    def _on_trail_toggle(self):
        try:
            self.application.controller.toggle_arm3d_trail(self._trail_var.get())
        except Exception:
            pass

    def _on_joint_ranges_toggle(self):
        try:
            self.application.controller.toggle_arm3d_joint_ranges(self._joint_ranges_var.get())
        except Exception:
            pass

    def _sync_sliders_from_model(self):
        """Actualiza los sliders y etiquetas con los ángulos actuales del modelo."""
        try:
            import graphics.layers as _layers
            layer = self.application.controller.robot_layer
            if isinstance(layer, _layers.Arm3DLayer):
                joints = layer.motor3d.model.joints
                for i, slider in enumerate(self._sliders):
                    if i < len(joints):
                        val = int(joints[i] + 90.0)
                        slider.set(val)
                        if i < len(self._val_labels):
                            self._val_labels[i].config(text=f"{val:>3}°")
        except Exception:
            pass

    def update_joint_limits(self):
        """Actualiza los rangos de los sliders con los límites del modelo cargado.
        Llamado desde MainApplication._connect_arm3d_info_panel() tras cargar el Arm3DLayer."""
        try:
            import graphics.layers as _layers
            layer = self.application.controller.robot_layer
            if not isinstance(layer, _layers.Arm3DLayer):
                return
            limits = layer.motor3d.model.joint_limits  # lista de (min, max) en grados DH
            for i, slider in enumerate(self._sliders):
                if i < len(limits):
                    lim_min = int(limits[i][0] + 90.0)
                    lim_max = int(limits[i][1] + 90.0)
                    slider.configure(from_=lim_min, to=lim_max)
                    # Re-clampear valor actual al nuevo rango
                    current = slider.get()
                    clamped = max(lim_min, min(lim_max, current))
                    if current != clamped:
                        slider.set(clamped)
        except Exception:
            pass


class Arm3DInfoPanel(tk.Frame):
    """Panel izquierdo de información en tiempo real del brazo robótico 3D.
    Muestra coordenadas XYZ del efector final, ángulos articulares J1-J6 con
    sus límites y estado de seguridad.
    Solo visible cuando el robot Braccio (opción 5) está activo.
    """

    def __init__(self, parent, *args, **kwargs):
        kwargs.setdefault('bg', DARK_BLUE)
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.configure(width=210)
        self.pack_propagate(False)

        # Título
        tk.Label(self, text="Brazo Robótico 3D",
                 bg=DARK_BLUE, fg="white",
                 font=("Consolas", 10, "bold")).pack(pady=(10, 4), padx=6, fill=tk.X)

        tk.Frame(self, bg=BLUE, height=1).pack(fill=tk.X, padx=4)

        # --- Efector final ---
        tk.Label(self, text="Efector Final",
                 bg=DARK_BLUE, fg="#00E5CC",
                 font=("Consolas", 9, "bold")).pack(anchor="w", padx=10, pady=(0, 2))

        ee_frame = tk.Frame(self, bg=DARK_BLUE)
        ee_frame.pack(fill=tk.X, padx=8)

        self._lbl_ee = {}
        for axis in ["X", "Y", "Z"]:
            row = tk.Frame(ee_frame, bg=DARK_BLUE)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=f"{axis}:", bg=DARK_BLUE, fg="white",
                     font=("Consolas", 9), width=3, anchor="e").pack(side=tk.LEFT)
            val = tk.Label(row, text="--- mm", bg=DARK_BLUE, fg="#00E5CC",
                           font=("Consolas", 9), anchor="w")
            val.pack(side=tk.LEFT, padx=4)
            self._lbl_ee[axis] = val

        tk.Frame(self, bg=BLUE, height=1).pack(fill=tk.X, padx=4, pady=6)

        # --- Articulaciones ---
        tk.Label(self, text="Articulaciones",
                 bg=DARK_BLUE, fg="white",
                 font=("Consolas", 9, "bold")).pack(anchor="w", padx=10, pady=(0, 2))

        joints_frame = tk.Frame(self, bg=DARK_BLUE)
        joints_frame.pack(fill=tk.X, padx=8)

        self._lbl_joints = []
        self._lbl_joint_limits = []
        for i in range(6):
            row = tk.Frame(joints_frame, bg=DARK_BLUE)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=f"J{i + 1}:", bg=DARK_BLUE, fg="white",
                     font=("Consolas", 9), width=3, anchor="e").pack(side=tk.LEFT)
            val = tk.Label(row, text="---°", bg=DARK_BLUE, fg="white",
                           font=("Consolas", 9), width=5, anchor="w")
            val.pack(side=tk.LEFT, padx=2)
            lim = tk.Label(row, text="[---,---]", bg=DARK_BLUE, fg="#AACCFF",
                           font=("Consolas", 8), anchor="w")
            lim.pack(side=tk.LEFT, padx=2)
            self._lbl_joints.append(val)
            self._lbl_joint_limits.append(lim)

        tk.Frame(self, bg=BLUE, height=1).pack(fill=tk.X, padx=4, pady=6)

        # --- Estado ---
        tk.Label(self, text="Estado",
                 bg=DARK_BLUE, fg="white",
                 font=("Consolas", 9, "bold")).pack(anchor="w", padx=10, pady=(0, 2))

        self._lbl_status = tk.Label(self, text="OK",
                                    bg=DARK_BLUE, fg="#aaffaa",
                                    font=("Consolas", 9), wraplength=195,
                                    justify=tk.LEFT, anchor="w")
        self._lbl_status.pack(anchor="w", padx=10, pady=2)

    def update(self, dof, joints, end_effector, in_workspace, singular,
               safety_blocked, warning_message, joint_limits=None):
        """Actualiza los valores mostrados. Llamado desde Arm3DHUD.update()."""
        # Efector actual
        if end_effector and len(end_effector) >= 3:
            for axis, val in zip(["X", "Y", "Z"], end_effector):
                self._lbl_ee[axis].config(text=f"{val:.0f} mm")
        else:
            for axis in ["X", "Y", "Z"]:
                self._lbl_ee[axis].config(text="--- mm")

        # Articulaciones con límites
        for i, (val_lbl, lim_lbl) in enumerate(zip(self._lbl_joints, self._lbl_joint_limits)):
            if joints and i < len(joints):
                val_lbl.config(text=f"{joints[i]:.0f}°")
            else:
                val_lbl.config(text="---°")
            if joint_limits and i < len(joint_limits):
                mn, mx = joint_limits[i]
                lim_lbl.config(text=f"[{mn:.0f}°,{mx:.0f}°]")
            else:
                lim_lbl.config(text="[---,---]")

        # Estado
        if safety_blocked:
            self._lbl_status.config(
                text=f"⚠ {warning_message or 'BLOQUEADO'}", fg="#FF6666")
        elif singular:
            self._lbl_status.config(text="⚠ Singularidad", fg="#FFAA00")
        elif not in_workspace:
            self._lbl_status.config(text="⚠ Fuera de rango", fg="#FFAA00")
        else:
            self._lbl_status.config(text="OK", fg="#aaffaa")


class DrawingFrame(tk.Frame):

    def __init__(self, parent, application: MainApplication = None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        self.application = application
        self.__load_images()

        self.hud_canvas = tk.Canvas(
            self, height=100, bg=DARK_BLUE, highlightthickness=1, highlightbackground="black")

        self.canvas_frame = tk.Frame(self, bg=BLUE)
        self.canvas = tk.Canvas(self.canvas_frame, bg="white", bd=1,
                                relief=tk.SOLID, highlightthickness=0)
        self.joystick_frame = JoystickFrame(self.canvas_frame, application, bg=DARK_BLUE, highlightthickness=1,
                                            highlightbackground="black")
        self.buttons_gamification = ButtonsGamification(self.canvas_frame, application, bg=DARK_BLUE,
                                                        highlightthickness=1, highlightbackground="black")

        # Botones de preset de cámara 3D [F] [L] [I] [Libre] — visibles solo para Braccio
        self.cam_buttons_frame = tk.Frame(self.canvas_frame, bg=DARK_BLUE,
                                          highlightthickness=1, highlightbackground=BLUE)
        for _text, _view in [("F", "front"), ("L", "side"), ("I", "iso"), ("Libre", None)]:
            tk.Button(
                self.cam_buttons_frame,
                text=_text, bg=DARK_BLUE, fg="white",
                font=("Consolas", 9, "bold"), bd=0, padx=8, pady=3,
                activebackground=BLUE, cursor="hand2",
                command=lambda v=_view: self._on_cam_preset(v)
            ).pack(side=tk.LEFT, padx=2, pady=2)

        self.bottom_frame = tk.Frame(self, bg=BLUE)
        self.key_movement = tk.Checkbutton(self.bottom_frame, text="Movimiento con el teclado", fg="white",
                                           font=("Consolas", 12),
                                           bg=BLUE, activebackground=BLUE, selectcolor="black",
                                           command=application.toggle_keys, underline=0)
        self.key_drawing = tk.Checkbutton(self.bottom_frame, text="Dibujar", fg="white",
                                          font=("Consolas", 12),
                                         bg=BLUE, activebackground=BLUE, selectcolor="black",
                                         command=application.set_drawing, underline=0)
        self.zoom_frame = tk.Frame(self.bottom_frame, bg=BLUE)
        self.zoom_in_button = ImageButton(
            self.zoom_frame,
            {
                "blue": self.zoom_img,
                "white": self.zoom_whi_img,
                "yellow": self.zoom_yel_img
            },
            bg=BLUE,
            bd=0
        )
        self.zoom_label = tk.Label(
            self.zoom_frame, bg=BLUE, fg="white", font=("Consolas", 12))
        self.zoom_out_button = ImageButton(
            self.zoom_frame,
            {
                "blue": self.dezoom_img,
                "white": self.dezoom_whi_img,
                "yellow": self.dezoom_yel_img
            },
            bg=BLUE,
            bd=0
        )

        self.canvas.bind("<ButtonPress-1>", self.press)
        self.canvas.bind("<ButtonRelease-1>", self.release)
        self.canvas.bind("<B1-Motion>", self.move)
        self.canvas.bind("<ButtonPress-3>", self.press_right)
        self.canvas.bind("<B3-Motion>", self.pan)
        self.canvas.bind("<MouseWheel>", self.zoom)
        application.bind("<Alt-m>", self.__toggle_check_manually)
        self.zoom_in_button.configure(command=self.application.zoom_in)
        self.zoom_out_button.configure(command=self.application.zoom_out)
        self.key_movement.select()

        self.zoom_in_button.grid(row=0, column=0, padx=5, pady=5)
        self.zoom_label.grid(row=0, column=1, padx=5, pady=5)
        self.zoom_out_button.grid(row=0, column=2, padx=5, pady=5)

        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.key_movement.pack(anchor="w", side=tk.LEFT)
        self.key_drawing.pack(anchor="w", side=tk.LEFT)
        self.zoom_frame.pack(anchor="e", side=tk.RIGHT)

        # Layout de DrawingFrame con grid (permite show/hide de hud_canvas sin cambiar el orden)
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.hud_canvas.grid(row=0, column=0, sticky="ew")
        self.canvas_frame.grid(row=1, column=0, sticky="nsew")
        self.bottom_frame.grid(row=2, column=0, sticky="ew")

        self.init_x = 0
        self.init_y = 0

    def __toggle_check_manually(self, event=None):
        self.key_movement.toggle()
        self.application.toggle_keys()

    def press(self, event):
        import graphics.layers as _layers
        layer = self.application.controller.robot_layer
        if isinstance(layer, _layers.Arm3DLayer):
            # El brazo 3D maneja el arrastre en move()
            self.canvas.focus_force()
            self.init_x = event.x
            self.init_y = event.y
            return
        if isinstance(layer.robot, robots.ArduinoBoard):
            layer.draw_component(event.x, event.y)
        self.canvas.focus_force()
        self.init_x = event.x
        self.init_y = event.y
        self.canvas.scan_mark(event.x, event.y)

    def release(self, event):
        import graphics.layers as _layers
        layer = self.application.controller.robot_layer
        if isinstance(layer, _layers.Arm3DLayer):
            return
        layer.drawing.dx += self.init_x - event.x
        layer.drawing.dy += self.init_y - event.y
        if layer.drawing.dx < 0:
            layer.drawing.dx = 0
        if layer.drawing.dx > 545:
            layer.drawing.dx = 545
        if layer.drawing.dy < 0:
            layer.drawing.dy = 0
        if layer.drawing.dy > 580:
            layer.drawing.dy = 580

    def move(self, event):
        import graphics.layers as _layers
        layer = self.application.controller.robot_layer
        if isinstance(layer, _layers.Arm3DLayer):
            dx = event.x - self.init_x
            dy = event.y - self.init_y
            self.init_x = event.x
            self.init_y = event.y
            self.application.controller.drag_arm3d_camera(dx, dy)
            return
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    def press_right(self, event):
        import graphics.layers as _layers
        if isinstance(self.application.controller.robot_layer, _layers.Arm3DLayer):
            self.canvas.focus_force()
            self.init_x = event.x
            self.init_y = event.y

    def pan(self, event):
        import graphics.layers as _layers
        if isinstance(self.application.controller.robot_layer, _layers.Arm3DLayer):
            dx = event.x - self.init_x
            dy = event.y - self.init_y
            self.init_x = event.x
            self.init_y = event.y
            self.application.controller.drag_arm3d_camera(dx, dy, pan=True)

    def zoom(self, event):
        if event.delta == -120:
            self.application.controller.zoom_out()
        elif event.delta == 120:
            self.application.controller.zoom_in()

    def change_zoom_label(self, zoom_level):
        self.zoom_label.configure(text="{}%".format(zoom_level))

    def show_joystick(self):
        self.joystick_frame.pack(anchor="center", fill=tk.X)

    def hide_joystick(self):
        self.joystick_frame.pack_forget()

    def show_button_keys_movement(self):
        self.key_movement.pack(anchor="w", side=tk.LEFT)

    def hide_button_keys_movement(self):
        self.key_movement.pack_forget()

    def show_buttons_gamification(self):
        self.buttons_gamification.pack(anchor="center", fill=tk.X)

    def hide_buttons_gamification(self):
        self.buttons_gamification.pack_forget()

    def show_hud(self):
        """Muestra el HUD strip superior (canvas de estado 2D)."""
        self.hud_canvas.grid()

    def hide_hud(self):
        """Oculta el HUD strip superior (usado cuando el panel de info lateral está activo)."""
        self.hud_canvas.grid_remove()

    def show_arm3d_camera_buttons(self):
        """Muestra los botones de preset de cámara 3D en la esquina inferior derecha del canvas."""
        self.cam_buttons_frame.place(relx=1.0, rely=1.0, anchor="se", x=-5, y=-5)

    def hide_arm3d_camera_buttons(self):
        """Oculta los botones de preset de cámara 3D."""
        self.cam_buttons_frame.place_forget()

    def _on_cam_preset(self, view_name):
        """Callback de los botones de preset de cámara."""
        self.application.controller.set_arm3d_camera_view(view_name)

    def __load_images(self):
        self.zoom_img = tk.PhotoImage(file="buttons/zoom.png")
        self.zoom_whi_img = tk.PhotoImage(file="buttons/zoom_w.png")
        self.zoom_yel_img = tk.PhotoImage(file="buttons/zoom_y.png")
        self.dezoom_img = tk.PhotoImage(file="buttons/dezoom.png")
        self.dezoom_whi_img = tk.PhotoImage(file="buttons/dezoom_w.png")
        self.dezoom_yel_img = tk.PhotoImage(file="buttons/dezoom_y.png")


class ButtonsGamification(tk.Frame):

    def __init__(self, parent, application: MainApplication = None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.application = application

        self.button_tutorial = tk.Button(self, text="TUTORIAL", bg=BLUE, bd=0, fg=DARK_BLUE, font=("Consolas", 13))
        self.button_hints = tk.Button(self, text="PISTAS", bg=BLUE, bd=0, fg=DARK_BLUE, font=("Consolas", 13))
        self.button_delete = tk.Button(self, text="ELIMINAR", bg=BLUE, bd=0, fg=DARK_BLUE, font=("Consolas", 13))
        self.button_results = tk.Button(self, text="RESULTADOS", bg=BLUE, bd=0, fg=DARK_BLUE, font=("Consolas", 13))

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)

        self.button_tutorial.grid(row=1, column=0, padx=(0, 20))
        self.button_hints.grid(row=1, column=1, padx=(0, 20))
        self.button_delete.grid(row=1, column=2, padx=(0, 20))
        self.button_results.grid(row=1, column=3, padx=(0, 20))

        self.button_tutorial.bind("<ButtonPress>", self.show_tutorial)
        self.button_hints.bind("<ButtonPress>", self.show_help)
        self.button_delete.bind("<ButtonPress>", self.delete_elements)
        self.button_results.bind("<ButtonPress>", self.show_results)

    def show_tutorial(self, event):
        self.application.controller.show_tutorial()
        self.application.controller.console.logger.write_log('info',
                                                             "El usuario ha consultado el tutorial del desafío: "
                                                             + str(self.application.selector_bar.
                                                                   gamification_option_selector.current() + 1))

    def show_results(self, event):
        self.application.controller.show_results()

    def show_help(self, event):
        self.application.controller.show_help(self.application.selector_bar.gamification_option_selector.current())
        self.application.controller.console.logger.write_log('info', "El usuario ha consultado una pista del desafío: "
                                                             + str(self.application.selector_bar.
                                                                   gamification_option_selector.current() + 1))

    def delete_elements(self, event):
        self.application.controller.delete_elements()
        self.application.controller.console.logger.write_log('info', "El usuario ha borrado el progreso del desafío: "
                                                             + str(self.application.selector_bar.
                                                                   gamification_option_selector.current() + 1))


class JoystickFrame(tk.Frame):

    def __init__(self, parent, application: MainApplication = None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.application = application

        self.lb_joystick = tk.Label(
            self, text="Joystick", bg=DARK_BLUE, fg="white", font=("Consolas", 13))
        self.lb_x = tk.Label(self, text="X:", bg=DARK_BLUE,
                             fg="white", font=("Consolas", 12))
        self.x_dir = tk.Scale(self, from_=0, to=1023, orient=tk.HORIZONTAL, bg=DARK_BLUE, fg="white",
                              sliderrelief=tk.FLAT, highlightthickness=0)
        self.lb_y = tk.Label(self, text="Y:", bg=DARK_BLUE,
                             fg="white", font=("Consolas", 12))
        self.y_dir = tk.Scale(self, from_=0, to=1023, orient=tk.HORIZONTAL, bg=DARK_BLUE, fg="white",
                              sliderrelief=tk.FLAT, highlightthickness=0)
        self.j_button = tk.Button(
            self, text="Botón", bg=BLUE, bd=0, fg=DARK_BLUE, font=("Consolas", 13))

        self.x_dir.set(500)
        self.y_dir.set(500)

        self.x_dir.bind("<ButtonRelease-1>", self.__updatex)
        self.y_dir.bind("<ButtonRelease-1>", self.__updatey)
        self.j_button.bind("<ButtonPress>", self.__pressb)
        self.j_button.bind("<ButtonRelease>", self.__releaseb)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)
        self.columnconfigure(4, weight=1)
        self.columnconfigure(5, weight=1)

        self.lb_joystick.grid(row=1, column=0, padx=(0, 20))
        self.lb_x.grid(row=1, column=1, padx=(0, 5))
        self.x_dir.grid(row=1, column=2, padx=(0, 20), pady=5)
        self.lb_y.grid(row=1, column=3, padx=(0, 5))
        self.y_dir.grid(row=1, column=4, padx=(0, 20), pady=5)
        self.j_button.grid(row=1, column=5, padx=10)

    def __updatex(self, event):
        self.application.controller.update_joystick('dx', self.x_dir.get())

    def __updatey(self, event):
        self.application.controller.update_joystick('dy', self.y_dir.get())

    def __pressb(self, event):
        self.application.controller.update_joystick('button', 0)

    def __releaseb(self, event):
        self.application.controller.update_joystick('button', 1)


class EditorFrame(tk.Frame):

    def __init__(self, parent, application: MainApplication = None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        self.text = self.TextEditor(self, bd=1, relief=tk.SOLID, wrap="none", font=("consolas", 12), undo=True,
                                    autoseparators=True)
        self.line_bar = self.LineNumberBar(
            self, width=30, bg=BLUE, bd=0, highlightthickness=0)
        self.sb_x = tk.Scrollbar(self, orient=tk.HORIZONTAL,
                                 command=self.text.xview)
        self.sb_y = tk.Scrollbar(self, orient=tk.VERTICAL,
                                 command=self.text.yview)

        self.create_file()

        self.text.update_highlight()
        self.line_bar.attach(self.text)
        self.text.config(xscrollcommand=self.sb_x.set,
                         yscrollcommand=self.sb_y.set)

        self.line_bar.grid(row=0, column=0, sticky="nsw")
        self.text.grid(row=0, column=1, sticky="nsew")
        self.sb_x.grid(row=1, column=1, sticky="sew")
        self.sb_y.grid(row=0, column=2, sticky="nse")

        self.text.bind("<<Change>>", self._on_change)
        self.text.bind("<Configure>", self._on_change)

        self.rowconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)

    def create_file(self):
        self.text.delete("1.0", tk.END)
        self.text.insert(tk.END, "void setup(){\n")
        self.text.insert(tk.END, "}\n\n")
        self.text.insert(tk.END, "void loop(){\n")
        self.text.insert(tk.END, "}")

    def change_text(self, text):
        self.text.delete("1.0", tk.END)
        self.text.insert(tk.END, text)

    def _on_change(self, event):
        self.line_bar.show_lines()

    class TextEditor(tk.Text):

        def __init__(self, *args, **kwargs):
            tk.Text.__init__(self, *args, **kwargs)

            self.keywords = self.__get_keywords("assets/colors.txt")
            self.comment_lines = []

            self._orig = self._w + "_orig"
            self.tk.call("rename", self._w, self._orig)
            self.tk.createcommand(self._w, self._proxy)
            self.__create_tags()

        def update_highlight(self):
            def step():
                i = 0
                while True:
                    keyword = self.keywords[i]
                    if len(keyword) == 3:
                        self.highlight_all_delimited(
                            r'%s' % keyword[0], r'%s' % keyword[1], keyword[2])
                    elif keyword[0][0:2] == '\\"' or keyword[0][0:2] == "\\'":
                        self.highlight_all(r'%s' % keyword[0], keyword[1], True)
                    elif keyword[0][0] == '\\':
                        self.highlight_all(r'%s' % keyword[0], keyword[1])
                    elif keyword[0][0:2] == '//':
                        self.highlight_all(r'%s' % keyword[0], keyword[1], True)
                    else:
                        self.highlight_all(r'\b%s\b' % keyword[0], keyword[1])
                    self.after(1, gen.__next__)
                    i += 1
                    if i == len(self.keywords):
                        i = 0
                        for match in self.comment_lines:
                            self.__remove_tags(match[0], match[1])
                        self.comment_lines = []
                    yield

            gen = step()
            gen.__next__()

        def highlight_all(self, pattern, tag, is_repaintable=False):
            for match in self.search_re(pattern, is_repaintable):
                self.highlight(tag, match[0], match[1])

        def highlight_all_delimited(self, pattern_s, pattern_e, tag):
            for match in self.search_re_delimited(pattern_s, pattern_e):
                self.highlight(tag, match[0], match[1])

        def highlight(self, tag, start, end):
            is_comment = False
            for comm in self.comment_lines:
                comm_start = tuple(map(int, comm[0].split('.')))
                comm_end = tuple(map(int, comm[1].split('.')))
                fl_start = tuple(map(int, start.split('.')))
                fl_end = tuple(map(int, end.split('.')))
                if (
                        comm_start[0] <= fl_start[0] <= comm_end[0]
                        and comm_start[0] <= fl_end[0] <= comm_end[0]
                ):
                    finished = False
                    if comm_start[0] == fl_start[0] == comm_end[0]:
                        if fl_end[0] == fl_start[0]:
                            finished = (
                                    comm_start[1] < fl_start[1] < comm_end[1]
                                    and comm_start[1] < fl_end[1] < comm_end[1]
                            )
                        else:
                            finished = True
                    elif comm_start[0] == fl_start[0]:
                        finished = comm_start[1] < fl_start[1]
                    elif comm_end[0] == fl_start[0]:
                        finished = fl_start[1] < comm_end[1]
                    elif comm_start[0] == fl_end[0]:
                        finished = comm_start[1] < fl_end[1]
                    elif comm_end[0] == fl_end[0]:
                        finished = fl_end[1] < comm_end[1]
                    else:
                        finished = True
                    if finished:
                        is_comment = True
                        break
            if not is_comment:
                self.__remove_tags(start, end)
                self.tag_add(tag, start, end)

        def search_re(self, pattern, is_repaintable):
            """
            Uses the python re library to match patterns.

            Arguments:
                pattern - The pattern to match.
            Return value:
                A list of tuples containing the start and end indices of the matches.
                e.g. [("0.4", "5.9"]
            """
            matches = []
            text = self.get("1.0", tk.END).splitlines()
            for i, line in enumerate(text):
                for match in re.finditer(pattern, line):
                    matches.append(
                        (f"{i + 1}.{match.start()}", f"{i + 1}.{match.end()}"))
            if is_repaintable:
                self.comment_lines.extend(matches)
            return matches

        def search_re_delimited(self, pattern_s, pattern_e):
            """
            Uses the python re library to match patterns.

            Arguments:
                pattern_s - The starting pattern to match.
                pattern_e - The ending pattern to match.
            Return value:
                A list of tuples containing the start and end indices of the matches.
                e.g. [("0.4", "5.9"]
            """
            matches = []
            text = self.get("1.0", tk.END).splitlines()
            start = end = -1
            for i, line in enumerate(text):
                for match in re.finditer(pattern_s, line):
                    if start == -1:
                        start = f"{i + 1}.{match.start()}"
                for match in re.finditer(pattern_e, line):
                    if end == -1:
                        end = f"{i + 1}.{match.end()}"
                if start != -1 and end != -1:
                    matches.append((start, end))
                    start = end = -1
            self.comment_lines = matches
            return matches

        def _proxy(self, *args):
            result = None
            command = (self._orig,) + args
            try:
                result = self.tk.call(command)
            except Exception:
                return result

            if (args[0] in ("insert", "replace", "delete")
                    or args[0:3] == ("mark", "set", "insert")
                    or args[0:2] == ("xview", "moveto")
                    or args[0:2] == ("xview", "scroll")
                    or args[0:2] == ("yview", "moveto")
                    or args[0:2] == ("yview", "scroll")):
                self.event_generate("<<Change>>", when="tail")

            return result

        def __create_tags(self):
            self.tag_configure("blue", foreground="#00979C")
            self.tag_configure("strblue", foreground="#005C5F")
            self.tag_configure("orange", foreground="#D35400")
            self.tag_configure("green", foreground="#728E00")
            self.tag_configure("gray", foreground="#95A5A6")
            self.tag_configure("dark", foreground="#434F54")

        def __remove_tags(self, start, end):
            self.tag_remove("blue", start, end)
            self.tag_remove("strblue", start, end)
            self.tag_remove("orange", start, end)
            self.tag_remove("green", start, end)
            self.tag_remove("gray", start, end)
            self.tag_remove("dark", start, end)

        def __get_keywords(self, file_name):
            keywords = []
            file = open(file_name, "r")
            lines = list(
                filter(lambda line: line != '',
                       map(lambda line: str(line).rstrip(), file.readlines())
                       )
            )
            for line in lines:
                line_elems = line.split('\t')
                if len(line_elems) == 2:
                    keywords.append((line_elems[0], line_elems[1]))
                elif len(line_elems) == 3:
                    keywords.append(
                        (line_elems[0], line_elems[1], line_elems[2]))
            return keywords

    class LineNumberBar(tk.Canvas):

        def __init__(self, *args, **kwargs):
            tk.Canvas.__init__(self, *args, **kwargs)
            self.editor = None

        def attach(self, editor):
            self.editor = editor

        def show_lines(self, *args):
            self.delete("all")

            i = self.editor.index("@0,0")
            while True:
                dline = self.editor.dlineinfo(i)
                if dline is None:
                    break
                line = str(i).split(".")[0]
                x = 28 - 9 * len(line)
                y = dline[1]
                self.create_text(x, y, anchor="nw", text=line,
                                 fill="white", font=('consolas', 12, 'bold'))
                i = self.editor.index("%s+1line" % i)


class ConsoleFrame(tk.Frame):

    def __init__(self, parent, application: MainApplication = None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.application = application

        self.output = tk.IntVar()
        self.warning = tk.IntVar()
        self.error = tk.IntVar()

        self.console_frame = tk.Frame(self, bg=DARK_BLUE)
        self.console = tk.Text(self.console_frame, bd=1, relief=tk.SOLID, font=(
            "consolas", 12), bg="black", fg="white")
        self.sb_y = tk.Scrollbar(
            self.console_frame, orient=tk.VERTICAL, command=self.console.yview)
        self.filter_frame = tk.Frame(self, bg=DARK_BLUE, padx=10)
        self.check_out = tk.Checkbutton(self.filter_frame, text="Output", fg="white", font=("Consolas", 12),
                                        bg=DARK_BLUE, activebackground=DARK_BLUE, selectcolor="black",
                                        variable=self.output, onvalue=1, offvalue=0,
                                        command=application.console_filter, underline=1)
        self.check_warning = tk.Checkbutton(self.filter_frame, text="Warning", fg="white", font=("Consolas", 12),
                                            bg=DARK_BLUE, activebackground=DARK_BLUE, selectcolor="black",
                                            variable=self.warning, onvalue=1, offvalue=0,
                                            command=application.console_filter, underline=0)
        self.check_error = tk.Checkbutton(self.filter_frame, text="Error", fg="white", font=("Consolas", 12),
                                          bg=DARK_BLUE, activebackground=DARK_BLUE, selectcolor="black",
                                          variable=self.error, onvalue=1, offvalue=0,
                                          command=application.console_filter, underline=3)
        self.input_frame = tk.Frame(self, bg=DARK_BLUE)
        self.input_entry = tk.Entry(self.input_frame, bd=1, relief=tk.SOLID, bg="black", insertbackground="white",
                                    fg="white", font=("Consolas", 12))
        self.input_button = tk.Button(self.input_frame, bd=0, bg=BLUE, fg=DARK_BLUE, text="Enviar",
                                      font=("Consolas", 12), command=self.__send_input, underline=0)

        self.console.config(state=tk.DISABLED, yscrollcommand=self.sb_y.set)
        self.check_out.select()
        self.check_warning.select()
        self.check_error.select()

        self.check_out.grid(column=0, row=0)
        self.check_warning.grid(column=0, row=1)
        self.check_error.grid(column=0, row=2)

        self.input_button.pack(side=tk.RIGHT, padx=(5, 0))
        self.input_entry.pack(fill=tk.X, expand=True)

        self.sb_y.pack(fill=tk.Y, side=tk.RIGHT)
        self.console.pack(fill=tk.BOTH, expand=True)

        self.filter_frame.pack(side=tk.RIGHT)
        self.input_frame.pack(fill=tk.X, side=tk.BOTTOM, expand=True, pady=5)
        self.console_frame.pack(fill=tk.BOTH, expand=True)

        self.application.bind("<Alt-u>", self.change_output)
        self.application.bind("<Alt-w>", self.change_warning)
        self.application.bind("<Alt-o>", self.change_error)
        self.application.bind("<Alt-e>", lambda event: self.__send_input())

    def change_output(self, event=None):
        self.check_out.toggle()
        self.__filter()

    def change_warning(self, event=None):
        self.check_warning.toggle()
        self.__filter()

    def change_error(self, event=None):
        self.check_error.toggle()
        self.__filter()

    def __filter(self):
        self.application.console_filter()

    def __send_input(self):
        self.application.controller.send_input(self.input_entry.get())


class ButtonBar(tk.Frame):

    def __init__(self, parent, application: MainApplication = None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.application = application

        self.exec_frame = tk.Frame(self, bg=kwargs["bg"])
        self.hist_frame = tk.Frame(self, bg=kwargs["bg"])
        self.utils_frame = tk.Frame(self, bg=kwargs["bg"])
        self.tooltip_hover = tk.Label(
            self, bg=kwargs["bg"], font=("consolas", 12), fg="white",
            width=18, anchor="w")

        self.__load_images()

        _bg = kwargs["bg"]

        def _imgbtn(parent, base, **kw):
            return ImageButton(
                parent,
                images={
                    "blue":   getattr(self, f"{base}_img"),
                    "white":  getattr(self, f"{base}_whi_img"),
                    "yellow": getattr(self, f"{base}_yel_img"),
                },
                bg=_bg, activebackground=DARK_BLUE, bd=0, **kw
            )

        # --- exec group (order matches mockup: Play Pause Stop StepBack StepFwd Reset) ---
        self.execute_button    = _imgbtn(self.exec_frame, "exec")
        self.pause_button      = _imgbtn(self.exec_frame, "pause")
        self.stop_button       = _imgbtn(self.exec_frame, "stop")
        self.step_back_button  = _imgbtn(self.exec_frame, "step_back")
        self.step_forward_button = _imgbtn(self.exec_frame, "step_forward")
        self.reset_button      = _imgbtn(self.exec_frame, "reset")

        # --- hist group ---
        self.undo_button = _imgbtn(self.hist_frame, "undo", command=self.application.editor_undo)
        self.redo_button = _imgbtn(self.hist_frame, "redo", command=self.application.editor_redo)

        # --- utils group ---
        self.save_button   = _imgbtn(self.utils_frame, "save",   command=self.application.save_file)
        self.import_button = _imgbtn(self.utils_frame, "import", command=self.application.open_file)

        # tooltips
        self.execute_button.set_tooltip_text(self.tooltip_hover, "Ejecutar")
        self.pause_button.set_tooltip_text(self.tooltip_hover, "Pausar / Reanudar")
        self.stop_button.set_tooltip_text(self.tooltip_hover, "Detener")
        self.step_back_button.set_tooltip_text(self.tooltip_hover, "Paso atrás")
        self.step_forward_button.set_tooltip_text(self.tooltip_hover, "Paso adelante")
        self.reset_button.set_tooltip_text(self.tooltip_hover, "Reiniciar")
        self.undo_button.set_tooltip_text(self.tooltip_hover, "Deshacer")
        self.redo_button.set_tooltip_text(self.tooltip_hover, "Rehacer")
        self.save_button.set_tooltip_text(self.tooltip_hover, "Guardar")
        self.import_button.set_tooltip_text(self.tooltip_hover, "Importar")

        self.execute_button.configure(command=self.execute)
        self.pause_button.configure(command=self.pause)
        self.stop_button.configure(command=self.stop)
        self.step_back_button.configure(command=self.step_back)
        self.step_forward_button.configure(command=self.step_forward)
        self.reset_button.configure(command=self.reset)

        self.status_badge = tk.Label(
            self,
            text="■  DETENIDO",
            bg="#555555", fg="white",
            font=("Consolas", 10, "bold"),
            padx=8, pady=3, relief="flat",
        )

        self.exec_frame.grid(row=0, column=0)
        self.hist_frame.grid(row=0, column=1)
        self.utils_frame.grid(row=0, column=2)
        self.status_badge.grid(row=0, column=3, padx=(12, 4))
        self.tooltip_hover.grid(row=0, column=4, padx=(4, 0))

        # exec group columns 0-5
        self.execute_button.grid(row=0, column=0, padx=5, pady=5)
        self.pause_button.grid(row=0, column=1, padx=5, pady=5)
        self.stop_button.grid(row=0, column=2, padx=5, pady=5)
        self.step_back_button.grid(row=0, column=3, padx=5, pady=5)
        self.step_forward_button.grid(row=0, column=4, padx=5, pady=5)
        self.reset_button.grid(row=0, column=5, padx=5, pady=5)

        self.undo_button.grid(row=0, column=0, padx=5, pady=5)
        self.redo_button.grid(row=0, column=1, padx=5, pady=5)
        self.save_button.grid(row=0, column=0, padx=5, pady=5)
        self.import_button.grid(row=0, column=1, padx=5, pady=5)

    def execute(self):
        self.execute_button.on_click()
        self.application.execute()
        self.execute_button.on_click_finish()

    def stop(self):
        self.stop_button.on_click()
        self.application.stop()
        self.stop_button.on_click_finish()

    def pause(self):
        self.application.toggle_pause()

    def step_forward(self):
        self.application.step_once()

    def step_back(self):
        self.application.step_back()

    def reset(self):
        self.reset_button.on_click()
        self.application.stop()
        # Defer execute() to the next event-loop tick so stop() finaliza primero
        self.application.after(0, self.application.execute)
        self.reset_button.on_click_finish()

    # ------------------------------------------------------------------
    # Estado visual del simulador
    # ------------------------------------------------------------------
    _STATE_CFG = {
        "idle":    ("■  DETENIDO",   "#555555", "white"),
        "running": ("▶  EJECUTANDO", "#007A3D", "white"),
        "paused":  ("⏸  PAUSADO",   "#B8860B", "white"),
    }

    def update_state(self, state: str):
        """Actualiza el badge de estado y el resaltado de botones."""
        text, bg, fg = self._STATE_CFG.get(state, self._STATE_CFG["idle"])
        self.status_badge.configure(text=text, bg=bg, fg=fg)
        self.execute_button.set_active(state == "running")
        self.pause_button.set_active(state == "paused")

    def __load_images(self):
        for name in ("exec", "import", "redo", "save", "stop", "undo",
                     "pause", "step_back", "step_forward", "reset"):
            setattr(self, f"{name}_img",     tk.PhotoImage(file=f"buttons/{name}.png"))
            setattr(self, f"{name}_whi_img", tk.PhotoImage(file=f"buttons/{name}_w.png"))
            setattr(self, f"{name}_yel_img", tk.PhotoImage(file=f"buttons/{name}_y.png"))


class ImageButton(tk.Button):

    def __init__(self, parent, images, *args, **kwargs):
        tk.Button.__init__(self, parent, *args, **kwargs, image=images["blue"])
        self.images = images

        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

    def on_enter(self, event):
        event.widget['image'] = self.images["white"]
        try:
            self.label.configure(text=self.tooltip)
        except AttributeError:
            pass

    def on_leave(self, event):
        event.widget['image'] = self.images["blue"]
        try:
            self.label.configure(text="")
        except AttributeError:
            pass

    def on_click(self):
        self.configure(image=self.images["yellow"])

    def on_click_finish(self):
        self.configure(image=self.images["blue"])

    def set_active(self, active: bool):
        """Mantiene el botón resaltado en amarillo mientras active=True."""
        self._active = active
        if active:
            self.configure(image=self.images["yellow"])
        else:
            self.configure(image=self.images["blue"])

    def on_leave(self, event):
        event.widget['image'] = self.images["yellow"] if getattr(self, '_active', False) else self.images["blue"]
        try:
            self.label.configure(text="")
        except AttributeError:
            pass

    def set_tooltip_text(self, label: tk.Label, tooltip):
        self.label = label
        self.tooltip = tooltip


class SelectorBar(tk.Frame):

    def __init__(self, parent, application: MainApplication = None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        self.lb_robot = tk.Label(self, text="Robot:", bg=DARK_BLUE, fg="white", font=(
            "Consolas", 13), underline=0)
        self.robot_selector = ttk.Combobox(self, state="readonly")
        self.lb_track = tk.Label(self, text="Circuito:", bg=DARK_BLUE, fg="white", font=(
            "Consolas", 13), underline=1)
        self.lb_gamification_option = tk.Label(self, text="Opción:", bg=DARK_BLUE, fg="white", font=(
            "Consolas", 13), underline=1)
        self.track_selector = ttk.Combobox(self, state="readonly")
        self.gamification_option_selector = ttk.Combobox(self, state="readonly")

        self.robot_selector['values'] = ["Robot móvil (2 infrarrojos)",
                                         "Robot móvil (3 infrarrojos)",
                                         "Robot móvil (4 infrarrojos)",
                                         "Actuador lineal",
                                         "Placa arduino",
                                         "Brazo Robótico (Braccio)"]
        self.robot_selector.current(0)
        self.track_selector['values'] = [
            "Circuito", "Laberinto", "Recta",
            "Obstáculo", "Recta y obstáculo",
            "Circuito con nodos"]
        self.track_selector.current(0)
        self.gamification_option_selector['values'] = [
            "Libre", "Desafío 1", "Desafío 2", "Desafío 3", "Desafío 4", "Desafío 5", "Desafío 6"]
        self.gamification_option_selector.current(0)

        self.robot_selector.bind(
            "<<ComboboxSelected>>", application.change_robot)
        self.track_selector.bind(
            "<<ComboboxSelected>>", application.change_track)
        self.gamification_option_selector.bind(
            "<<ComboboxSelected>>", application.change_gamification_option)
        application.bind("<Alt-r>", lambda event: self.robot_selector.focus())
        application.bind("<Alt-i>", lambda event: self.track_selector.focus())
        application.bind("<Alt-o>", lambda event: self.gamification_option_selector.focus())

        self.lb_robot.grid(row=0, column=0)
        self.robot_selector.grid(row=0, column=1, padx=(5, 15))
        self.lb_track.grid(row=0, column=2)
        self.track_selector.grid(row=0, column=3, padx=(5, 10))

    def hide_circuit_selector(self):
        if self.lb_track.winfo_ismapped():
            self.lb_track.grid_forget()
        if self.track_selector.winfo_ismapped():
            self.track_selector.grid_forget()

    def recover_circuit_selector(self):
        if not self.lb_track.winfo_ismapped():
            self.lb_track.grid(row=0, column=2)
        if not self.track_selector.winfo_ismapped():
            self.track_selector.grid(row=0, column=3, padx=(5, 10))

    def hide_gamification_option_selector(self):
        if self.lb_gamification_option.winfo_ismapped():
            self.lb_gamification_option.grid_forget()
        if self.gamification_option_selector.winfo_ismapped():
            self.gamification_option_selector.grid_forget()

    def recover_gamification_option_selector(self):
        if not self.lb_gamification_option.winfo_ismapped():
            self.lb_gamification_option.grid(row=0, column=2)
        if not self.gamification_option_selector.winfo_ismapped():
            self.gamification_option_selector.grid(row=0, column=3, padx=(5, 10))
