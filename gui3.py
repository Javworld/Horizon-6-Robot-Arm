
import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog
import json # Import para manejar archivos JSON
import serial
import serial.tools.list_ports
import threading
import queue
import time # NUEVO: Import para los retrasos en la secuencia

import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

ctk.set_appearance_mode("System")
ctk.set_default_color_theme("blue")

class SequencerWindow(ctk.CTkToplevel):
    def __init__(self, master, main_app):
        super().__init__(master)
        self.title("Secuenciador de Movimientos")
        self.geometry("800x400")
        
        self.main_app = main_app
        self.sequence = []
        self.is_sequence_running = False
        self.stop_sequence_flag = False

        # --- Panel de Secuenciador ---
        ctk.CTkLabel(self, text="Secuenciador de Movimientos", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=10)
        
        list_management_frame = ctk.CTkFrame(self, fg_color="transparent")
        list_management_frame.pack(fill="x", expand=True, padx=10)
        list_management_frame.grid_columnconfigure(0, weight=1)
        list_management_frame.grid_columnconfigure(1, weight=0)
        
        self.sequence_listbox = tk.Listbox(list_management_frame, height=10)
        self.sequence_listbox.grid(row=0, column=0, rowspan=5, padx=5, pady=5, sticky="nsew") # rowspan aumentado
        
        add_point_button = ctk.CTkButton(list_management_frame, text="Grabar Punto Actual", command=self.add_point_to_sequence)
        add_point_button.grid(row=0, column=1, padx=10, pady=5)
        delete_point_button = ctk.CTkButton(list_management_frame, text="Borrar Punto Seleccionado", command=self.delete_selected_point)
        delete_point_button.grid(row=1, column=1, padx=10, pady=5)
        clear_all_button = ctk.CTkButton(list_management_frame, text="Limpiar Secuencia", command=self.clear_sequence)
        clear_all_button.grid(row=2, column=1, padx=10, pady=5)

        # --- NUEVO: Botones para Guardar y Cargar ---
        save_button = ctk.CTkButton(list_management_frame, text="Guardar Secuencia", command=self.save_sequence_to_file)
        save_button.grid(row=3, column=1, padx=10, pady=(15, 5)) # Espacio extra arriba
        load_button = ctk.CTkButton(list_management_frame, text="Cargar Secuencia", command=self.load_sequence_from_file)
        load_button.grid(row=4, column=1, padx=10, pady=5)


        # ... (El resto de la interfaz del secuenciador y las funciones sin cambios) ...
        run_controls_frame = ctk.CTkFrame(self, fg_color="transparent")
        run_controls_frame.pack(fill="x", pady=10, padx=10)
        ctk.CTkLabel(run_controls_frame, text="Retraso entre puntos (s):").pack(side="left", padx=(0,5))
        self.sequence_delay_entry = ctk.CTkEntry(run_controls_frame, width=60)
        self.sequence_delay_entry.insert(0, "1.0")
        self.sequence_delay_entry.pack(side="left")
        self.run_sequence_button = ctk.CTkButton(run_controls_frame, text="EJECUTAR SECUENCIA", command=self.run_sequence, fg_color="green")
        self.run_sequence_button.pack(side="right", padx=5)
        self.stop_sequence_button = ctk.CTkButton(run_controls_frame, text="PARAR", command=self.stop_sequence, fg_color="red")
        self.stop_sequence_button.pack(side="right")


    # --- NUEVAS FUNCIONES PARA MANEJAR ARCHIVOS ---

    def save_sequence_to_file(self):
        """Abre un diálogo para guardar la secuencia actual en un archivo JSON."""
        if not self.sequence:
            self.main_app.log_to_console("Error: No hay secuencia para guardar.\n")
            return
            
        filepath = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("Archivos de Secuencia JSON", "*.json"), ("Todos los Archivos", "*.*")],
            title="Guardar Secuencia Como..."
        )
        
        if not filepath: # Si el usuario cancela el diálogo
            return
            
        try:
            with open(filepath, 'w') as f:
                json.dump(self.sequence, f, indent=4) # indent=4 para que sea legible
            self.main_app.log_to_console(f"Secuencia guardada exitosamente en: {filepath}\n")
        except Exception as e:
            self.main_app.log_to_console(f"Error al guardar el archivo: {e}\n")

    def load_sequence_from_file(self):
        """Abre un diálogo para cargar una secuencia desde un archivo JSON."""
        filepath = filedialog.askopenfilename(
            filetypes=[("Archivos de Secuencia JSON", "*.json"), ("Todos los Archivos", "*.*")],
            title="Abrir Secuencia"
        )

        if not filepath: # Si el usuario cancela
            return
            
        try:
            with open(filepath, 'r') as f:
                loaded_sequence = json.load(f)
            
            # Validación básica de la estructura del archivo
            if isinstance(loaded_sequence, list) and all(isinstance(item, dict) for item in loaded_sequence):
                self.sequence = loaded_sequence
                self.update_sequence_listbox()
                self.main_app.log_to_console(f"Secuencia cargada desde: {filepath}\n")
            else:
                self.main_app.log_to_console("Error: El archivo no tiene el formato de secuencia esperado.\n")

        except json.JSONDecodeError:
            self.main_app.log_to_console("Error: El archivo no es un JSON válido.\n")
        except Exception as e:
            self.main_app.log_to_console(f"Error al cargar el archivo: {e}\n")


    # ... (El resto de tus funciones de SequencerWindow: add_point, run_sequence, etc., sin cambios)
    def add_point_to_sequence(self):
        """Toma los ángulos y RPMs actuales del panel manual y los añade a la secuencia."""
        try:
            current_angles = [float(entry.get()) for entry in self.main_app.angle_entries]
            current_rpms = [int(entry.get()) for entry in self.main_app.rpm_entries]
            point_data = {"angles": current_angles, "rpms": current_rpms}
            self.sequence.append(point_data)
            self.update_sequence_listbox()
            self.main_app.log_to_console(f"Punto {len(self.sequence)} grabado.\n")
        except ValueError:
            self.main_app.log_to_console("Error: Todos los campos de ángulo/RPM deben ser números válidos.\n")
    def update_sequence_listbox(self):
        """Refresca el Listbox con la secuencia actual, mostrando ángulos y RPMs."""
        self.sequence_listbox.delete(0, tk.END)
        for i, point_data in enumerate(self.sequence):
            angles_str = ", ".join([f"{a:.1f}°" for a in point_data['angles']])
            rpms_str = ", ".join([f"{r}" for r in point_data['rpms']])
            display_str = f"Punto {i+1} -> Ángulos: [{angles_str}] | RPMs: [{rpms_str}]"
            self.sequence_listbox.insert(tk.END, display_str)
    def run_sequence(self):
        """Inicia la ejecución de la secuencia en un hilo separado."""
        if self.is_sequence_running:
            self.main_app.log_to_console("Error: La secuencia ya está en ejecución.\n")
            return
        if not self.sequence:
            self.main_app.log_to_console("Error: No hay puntos en la secuencia para ejecutar.\n")
            return
        self.is_sequence_running = True
        self.stop_sequence_flag = False
        self.run_sequence_button.configure(state="disabled")
        sequence_thread = threading.Thread(target=self._sequence_worker, daemon=True)
        sequence_thread.start()
    def _sequence_worker(self):
        """El hilo que ejecuta la secuencia punto por punto."""
        try:
            delay = float(self.sequence_delay_entry.get())
            self.main_app.log_to_console(f"--- Iniciando secuencia con {delay}s de retraso ---\n")
            for i, point_data in enumerate(self.sequence):
                if self.stop_sequence_flag:
                    break
                self.main_app.log_to_console(f"Moviendo a Punto {i+1}...\n")
                command_parts = []
                for j in range(6):
                    angle = point_data['angles'][j]
                    rpm = point_data['rpms'][j]
                    command_parts.append(f"J{j+1} {angle:.2f} {rpm}")
                command = "DMOVE " + " ".join(command_parts)
                self.main_app.send_command(command)
                time.sleep(5) 
                if self.stop_sequence_flag:
                    break
                time.sleep(delay)
            if not self.stop_sequence_flag:
                self.main_app.log_to_console("--- Secuencia completada exitosamente ---\n")
            else:
                 self.main_app.log_to_console("--- Secuencia detenida por el usuario ---\n")
        except ValueError:
            self.main_app.log_to_console("Error en el parámetro de Retraso.\n")
        finally:
            self.is_sequence_running = False
            self.run_sequence_button.configure(state="normal")
    def delete_selected_point(self):
        selected_indices = self.sequence_listbox.curselection()
        if not selected_indices:
            self.main_app.log_to_console("Error: No hay ningún punto seleccionado para borrar.\n")
            return
        for index in sorted(selected_indices, reverse=True):
            del self.sequence[index]
        self.update_sequence_listbox()
    def clear_sequence(self):
        self.sequence.clear()
        self.update_sequence_listbox()
        self.main_app.log_to_console("Secuencia limpiada.\n")
    def stop_sequence(self):
        if self.is_sequence_running:
            self.stop_sequence_flag = True
            self.main_app.log_to_console("Solicitud de parada enviada...\n")

class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Control Principal de Brazo Robótico")
        self.root.geometry("1000x950")

        # --- Variables de estado y referencias ---
        self.sequencer_window = None # Referencia a la ventana del secuenciador
        self.last_ik_solution = None # Para guardar la última solución IK en radianes
        self.serial_port = None
        self.serial_queue = queue.Queue()

        # --- Listas para guardar referencia a los widgets y datos ---
        self.angle_sliders = []
        self.angle_entries = []
        self.rpm_entries = []
        self.joint_labels = ["J1 (Base)", "J2 (Hombro)", "J3 (Codo)", "J4 (Muñeca V)", "J5 (Muñeca H)", "J6 (Pinza)"]
        self.joint_limits = [(-180, 180), (-90, 90), (-140, 140), (-180, 180), (-90, 90), (-180, 180)]

        # --- Integración del Modelo del Robot ---
        self.robot_model = self.define_robot_model()
        if self.robot_model:
            print("Modelo de robot 'Horizon6' cargado exitosamente.")
            print(self.robot_model)

        # --- Construcción de la GUI ---
        main_frame = ctk.CTkFrame(root)
        main_frame.pack(pady=10, padx=10, fill="both", expand=True)

        # --- Panel Superior (Conexión y Comandos Globales) ---
        top_frame = ctk.CTkFrame(main_frame, fg_color="transparent")
        top_frame.pack(fill="x", padx=10, pady=5)
        top_frame.grid_columnconfigure(0, weight=1)
        top_frame.grid_columnconfigure(1, weight=1)

        connection_frame = ctk.CTkFrame(top_frame)
        connection_frame.grid(row=0, column=0, padx=(0, 5), pady=10, sticky="ew")
        ctk.CTkLabel(connection_frame, text="Conexión", font=ctk.CTkFont(size=12, weight="bold")).pack()
        port_control_frame = ctk.CTkFrame(connection_frame, fg_color="transparent")
        port_control_frame.pack(pady=5, padx=10)
        self.ports_combobox = ctk.CTkComboBox(port_control_frame, values=[])
        self.ports_combobox.grid(row=0, column=0, padx=5)
        self.refresh_ports_button = ctk.CTkButton(port_control_frame, text="Refrescar Puertos", command=self.update_ports_list)
        self.refresh_ports_button.grid(row=0, column=1, padx=5)
        self.connect_button = ctk.CTkButton(port_control_frame, text="Conectar", command=self.connect_serial)
        self.connect_button.grid(row=0, column=2, padx=5)
        self.connection_status_label = ctk.CTkLabel(port_control_frame, text="Estado: Desconectado", text_color=("red", "salmon"))
        self.connection_status_label.grid(row=0, column=3, padx=10)

        main_commands_frame = ctk.CTkFrame(top_frame)
        main_commands_frame.grid(row=0, column=1, padx=(5, 0), pady=10, sticky="ew")
        ctk.CTkLabel(main_commands_frame, text="Comandos Globales", font=ctk.CTkFont(size=12, weight="bold")).pack()
        buttons_frame = ctk.CTkFrame(main_commands_frame, fg_color="transparent")
        buttons_frame.pack(pady=5)
        self.init_all_button = ctk.CTkButton(buttons_frame, text="INIT ALL", command=lambda: self.send_command("INITALL"))
        self.init_all_button.grid(row=0, column=0, padx=10, pady=5)
        self.home_all_button = ctk.CTkButton(buttons_frame, text="HOME ALL", command=lambda: self.send_command("HOME"))
        self.home_all_button.grid(row=0, column=1, padx=10, pady=5)
        self.status_all_button = ctk.CTkButton(buttons_frame, text="STATUS ALL", command=lambda: self.send_command("STATUSALL"))
        self.status_all_button.grid(row=0, column=2, padx=10, pady=5)
        self.open_sequencer_button = ctk.CTkButton(main_commands_frame, text="Abrir Secuenciador", command=self.open_sequencer_window)
        self.open_sequencer_button.pack(pady=10)

        # --- Panel de Comandos Manuales ---
        manual_command_frame = ctk.CTkFrame(main_frame)
        manual_command_frame.pack(pady=10, padx=10, fill="x")
        ctk.CTkLabel(manual_command_frame, text="Entrada de Comando Manual", font=ctk.CTkFont(size=16, weight="bold")).pack()
        sender_frame = ctk.CTkFrame(manual_command_frame, fg_color="transparent")
        sender_frame.pack(pady=5, padx=10, fill="x")
        self.manual_command_entry = ctk.CTkEntry(sender_frame, placeholder_text="Escribe un comando y presiona Enter...")
        self.manual_command_entry.pack(side="left", fill="x", expand=True, padx=(0,10))
        self.manual_command_entry.bind("<Return>", self.send_manual_command_event)
        send_button = ctk.CTkButton(sender_frame, text="Enviar Comando", command=self.send_manual_command)
        send_button.pack(side="left")

        # --- Panel de Control Manual de Ejes ---
        joint_control_frame = ctk.CTkFrame(main_frame)
        joint_control_frame.pack(pady=10, padx=10, fill="x")
        ctk.CTkLabel(joint_control_frame, text="Control Manual de Ejes (DMOVE)", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=10)
        for i in range(6):
            joint_frame = ctk.CTkFrame(joint_control_frame)
            joint_frame.pack(fill="x", padx=10, pady=5)
            label = ctk.CTkLabel(joint_frame, text=self.joint_labels[i], width=120, font=ctk.CTkFont(weight="bold"))
            label.pack(side="left", padx=5)
            slider = ctk.CTkSlider(joint_frame, from_=self.joint_limits[i][0], to=self.joint_limits[i][1], command=lambda value, index=i: self.update_angle_from_slider(value, index))
            slider.pack(side="left", fill="x", expand=True, padx=5)
            self.angle_sliders.append(slider)
            entry = ctk.CTkEntry(joint_frame, width=70)
            entry.pack(side="left", padx=5)
            entry.bind("<Return>", lambda event, index=i: self.update_angle_from_entry(index))
            self.angle_entries.append(entry)
            rpm_label = ctk.CTkLabel(joint_frame, text="RPM:")
            rpm_label.pack(side="left", padx=(10, 2))
            rpm_entry = ctk.CTkEntry(joint_frame, width=50)
            rpm_entry.insert(0, "15")
            rpm_entry.pack(side="left", padx=(0, 5))
            self.rpm_entries.append(rpm_entry)
            move_button = ctk.CTkButton(joint_frame, text="MOVER", width=60, command=lambda index=i: self.move_single_joint(index))
            move_button.pack(side="left", padx=5)
            self.update_angle_from_slider(0, i)
        global_move_frame = ctk.CTkFrame(joint_control_frame, fg_color="transparent")
        global_move_frame.pack(fill="x", padx=10, pady=10)
        self.move_all_button = ctk.CTkButton(global_move_frame, text="MOVER TODO A OBJETIVO", command=self.move_all_joints)
        self.move_all_button.pack(side="right", padx=10)
        
        # --- Panel de Cinemática Inversa ---
        ik_frame = ctk.CTkFrame(main_frame)
        ik_frame.pack(pady=10, padx=10, fill="x")
        ctk.CTkLabel(ik_frame, text="Cinemática Inversa (Posición XYZ, Orientación RPY)", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=10)
        target_frame = ctk.CTkFrame(ik_frame, fg_color="transparent")
        target_frame.pack(pady=5, padx=10)
        ctk.CTkLabel(target_frame, text="X (m):").grid(row=0, column=0, padx=5, pady=2)
        self.ik_x = ctk.CTkEntry(target_frame, width=70, placeholder_text="0.3")
        self.ik_x.grid(row=0, column=1)
        ctk.CTkLabel(target_frame, text="Y (m):").grid(row=0, column=2, padx=5, pady=2)
        self.ik_y = ctk.CTkEntry(target_frame, width=70, placeholder_text="0.2")
        self.ik_y.grid(row=0, column=3)
        ctk.CTkLabel(target_frame, text="Z (m):").grid(row=0, column=4, padx=5, pady=2)
        self.ik_z = ctk.CTkEntry(target_frame, width=70, placeholder_text="0.5")
        self.ik_z.grid(row=0, column=5)
        ctk.CTkLabel(target_frame, text="Roll (°):").grid(row=1, column=0, padx=5, pady=2)
        self.ik_roll = ctk.CTkEntry(target_frame, width=70, placeholder_text="0")
        self.ik_roll.grid(row=1, column=1)
        ctk.CTkLabel(target_frame, text="Pitch (°):").grid(row=1, column=2, padx=5, pady=2)
        self.ik_pitch = ctk.CTkEntry(target_frame, width=70, placeholder_text="0")
        self.ik_pitch.grid(row=1, column=3)
        ctk.CTkLabel(target_frame, text="Yaw (°):").grid(row=1, column=4, padx=5, pady=2)
        self.ik_yaw = ctk.CTkEntry(target_frame, width=70, placeholder_text="0")
        self.ik_yaw.grid(row=1, column=5)
        ik_button_frame = ctk.CTkFrame(ik_frame, fg_color="transparent")
        ik_button_frame.pack(pady=10)
        self.ik_calculate_button = ctk.CTkButton(ik_button_frame, text="Calcular Solución", command=self.calculate_ik_solution)
        self.ik_calculate_button.grid(row=0, column=0, padx=10)
        self.ik_move_button = ctk.CTkButton(ik_button_frame, text="Mover a Solución", command=self.move_to_ik_solution, state="disabled")
        self.ik_move_button.grid(row=0, column=1, padx=10)

        # --- Panel de la Consola ---
        console_frame = ctk.CTkFrame(main_frame)
        console_frame.pack(pady=10, padx=10, fill="both", expand=True)
        ctk.CTkLabel(console_frame, text="Consola del Arduino", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(5,0))
        self.console_text = ctk.CTkTextbox(console_frame, state='disabled', wrap=tk.WORD)
        self.console_text.pack(expand=True, fill='both', padx=5, pady=5)
        
        self.update_ports_list()
        self.root.after(100, self.process_serial_queue)

    def define_robot_model(self):
        try:
            d1, a2, d4, d6 = 265.5/1000, 225/1000, 184.8/1000, 72.21/1000
            L1 = RevoluteDH(d=d1, a=0, alpha=np.pi/2)
            L2 = RevoluteDH(d=0, a=a2, alpha=0, offset=np.pi/2)
            L3 = RevoluteDH(d=0, a=0, alpha=np.pi/2)
            L4 = RevoluteDH(d=d4, a=0, alpha=-np.pi/2)
            L5 = RevoluteDH(d=0, a=0, alpha=np.pi/2, offset=-np.pi/2)
            L6 = RevoluteDH(d=d6, a=0, alpha=0)
            L1.qlim, L2.qlim, L3.qlim, L4.qlim, L5.qlim, L6.qlim = (np.deg2rad(self.joint_limits[i]) for i in range(6))
            return DHRobot([L1, L2, L3, L4, L5, L6], name='Horizon6')
        except Exception as e:
            print(f"Error al cargar el modelo del robot: {e}")
            return None

    def open_sequencer_window(self):
        if self.sequencer_window is None or not self.sequencer_window.winfo_exists():
            self.sequencer_window = SequencerWindow(master=self.root, main_app=self)
            self.sequencer_window.protocol("WM_DELETE_WINDOW", self.on_sequencer_close)
        else:
            self.sequencer_window.focus()

    def on_sequencer_close(self):
        self.sequencer_window.destroy()
        self.sequencer_window = None

    def send_manual_command_event(self, event=None):
        self.send_manual_command()

    def send_manual_command(self):
        command = self.manual_command_entry.get()
        if command:
            self.send_command(command)
            self.manual_command_entry.delete(0, tk.END)
    
    def calculate_ik_solution(self):
        if not self.robot_model:
            self.log_to_console("Error: Modelo del robot no cargado.\n")
            return
        try:
            x = float(self.ik_x.get() or self.ik_x.cget("placeholder_text"))
            y = float(self.ik_y.get() or self.ik_y.cget("placeholder_text"))
            z = float(self.ik_z.get() or self.ik_z.cget("placeholder_text"))
            roll = float(self.ik_roll.get() or self.ik_roll.cget("placeholder_text"))
            pitch = float(self.ik_pitch.get() or self.ik_pitch.cget("placeholder_text"))
            yaw = float(self.ik_yaw.get() or self.ik_yaw.cget("placeholder_text"))
            self.log_to_console(f"Objetivo IK: X={x} Y={y} Z={z} R={roll} P={pitch} Y={yaw}\n")
            T_objetivo = SE3(x, y, z) * SE3.RPY([roll, pitch, yaw], order='xyz', unit='deg')
            self.log_to_console("Calculando solución de cinemática inversa...\n")
            solucion = self.robot_model.ikine_LM(T_objetivo, q0=self.get_current_angles_rad())
            if solucion.success:
                self.last_ik_solution = solucion.q
                q_deg = np.rad2deg(self.last_ik_solution)
                self.log_to_console(f"Solución encontrada (grados): {[f'{q:.2f}' for q in q_deg]}\n")
                for i in range(6):
                    self.angle_sliders[i].set(q_deg[i])
                    self.update_angle_from_slider(q_deg[i], i)
                self.ik_move_button.configure(state="normal")
            else:
                self.last_ik_solution = None
                self.ik_move_button.configure(state="disabled")
                self.log_to_console("Error: No se encontró una solución de cinemática inversa para el objetivo.\n")
        except Exception as e:
            self.log_to_console(f"Error en cálculo IK: {e}\n")

    def move_to_ik_solution(self):
        if self.last_ik_solution is None:
            self.log_to_console("Error: No hay una solución IK válida para mover.\n")
            return
        q_deg = np.rad2deg(self.last_ik_solution)
        command_parts = []
        for i in range(6):
            try:
                rpm = int(self.rpm_entries[i].get())
            except ValueError:
                rpm = 15
            command_parts.append(f"J{i+1} {q_deg[i]:.2f} {rpm}")
        command = "DMOVE " + " ".join(command_parts)
        self.log_to_console("Enviando movimiento a la solución IK calculada...\n")
        self.send_command(command)

    def get_current_angles_rad(self):
        try:
            deg_angles = [float(entry.get()) for entry in self.angle_entries]
            return np.deg2rad(deg_angles)
        except:
            return None

    def update_angle_from_slider(self, value, index):
        self.angle_entries[index].delete(0, tk.END)
        self.angle_entries[index].insert(0, f"{value:.2f}")

    def update_angle_from_entry(self, index):
        try:
            value = float(self.angle_entries[index].get())
            min_val, max_val = self.joint_limits[index]
            value = max(min_val, min(max_val, value))
            self.angle_sliders[index].set(value)
            self.angle_entries[index].delete(0, tk.END)
            self.angle_entries[index].insert(0, f"{value:.2f}")
        except ValueError:
            pass

    def move_single_joint(self, index):
        try:
            angle = float(self.angle_entries[index].get())
            rpm = int(self.rpm_entries[index].get())
            command = f"DMOVE J{index+1} {angle:.2f} {rpm}"
            self.send_command(command)
        except ValueError:
            self.log_to_console("Error: El ángulo y las RPM deben ser números válidos.\n")

    def move_all_joints(self):
        try:
            command_parts = []
            for i in range(6):
                angle = float(self.angle_entries[i].get())
                rpm = int(self.rpm_entries[i].get())
                command_parts.append(f"J{i+1} {angle:.2f} {rpm}")
            command = "DMOVE " + " ".join(command_parts)
            self.send_command(command)
        except ValueError:
            self.log_to_console("Error: Los ángulos y RPMs deben ser números válidos.\n")

    def update_ports_list(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.ports_combobox.configure(values=ports if ports else ["No se encontraron puertos"])
        if ports:
            self.ports_combobox.set(ports[0])

    def connect_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.stop_reading_thread = True
            if hasattr(self, 'read_thread') and self.read_thread.is_alive():
                self.read_thread.join()
            self.serial_port.close()
            self.connection_status_label.configure(text="Estado: Desconectado", text_color=("red", "salmon"))
            self.connect_button.configure(text="Conectar")
            self.log_to_console("--- Desconectado ---\n")
        else:
            port_name = self.ports_combobox.get()
            if not port_name or "No se encontraron" in port_name:
                self.log_to_console("Error: No se ha seleccionado ningún puerto válido.\n")
                return
            try:
                self.serial_port = serial.Serial(port_name, 9600, timeout=1)
                self.connection_status_label.configure(text=f"Estado: Conectado", text_color=("green", "lightgreen"))
                self.connect_button.configure(text="Desconectar")
                self.stop_reading_thread = False
                self.read_thread = threading.Thread(target=self.read_from_port, daemon=True)
                self.read_thread.start()
                self.log_to_console(f"--- Conectado a {port_name} ---\n")
            except serial.SerialException as e:
                self.log_to_console(f"Error al conectar: {e}\n")

    def read_from_port(self):
        while not self.stop_reading_thread:
            try:
                if self.serial_port and self.serial_port.is_open:
                    line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        self.serial_queue.put(line)
            except (serial.SerialException, TypeError):
                self.serial_queue.put("ERROR_DISCONNECTED")
                break
    
    def process_serial_queue(self):
        try:
            while not self.serial_queue.empty():
                message = self.serial_queue.get_nowait()
                if message == "ERROR_DISCONNECTED":
                    if self.serial_port and self.serial_port.is_open:
                        self.serial_port.close()
                    self.connection_status_label.configure(text="Estado: Desconectado (Error)", text_color=("red", "salmon"))
                    self.connect_button.configure(text="Conectar")
                    self.log_to_console("--- Conexión perdida ---\n")
                else:
                    self.log_to_console(message + '\n')
        finally:
            self.root.after(100, self.process_serial_queue)

    def send_command(self, command):
        if self.serial_port and self.serial_port.is_open:
            full_command = command + '\n'
            self.serial_port.write(full_command.encode('utf-8'))
            self.log_to_console(f"-> {command}\n")
        else:
            self.log_to_console("Error: No estás conectado a ningún puerto.\n")
            
    def log_to_console(self, message):
        self.console_text.configure(state='normal')
        self.console_text.insert(tk.END, message)
        self.console_text.configure(state='disabled')
        self.console_text.see(tk.END)


if __name__ == "__main__":
    root = ctk.CTk()
    app = RobotArmGUI(root)
    root.mainloop()