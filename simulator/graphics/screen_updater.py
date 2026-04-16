import time
import threading
import graphics.layers as layers


layer: layers.Layer = None
last_update = 0
view = None
controller = None  # Referencia al RobotsController, necesaria para debug_line()


def refresh():
    global layer
    global last_update
    global view
    if layer is None or view is None:
        return
    curr_time = time.time_ns() / 1000000
    if last_update + 16 <= curr_time:
        layer.move(view.keys_used, view.move_WASD)
        view.update_idletasks()
        last_update = time.time_ns() / 1000000


def update():
    """Alias de refresh() — llamado desde código generado en bucles for/do-while."""
    refresh()


def debug_line(line_no):
    """Llamado desde el código transpilado antes de cada sentencia.
    Cuando se ejecuta en el hilo principal refresca la pantalla y gestiona
    breakpoints. Desde un hilo de fondo se omiten las operaciones de Tkinter."""
    if threading.current_thread() is threading.main_thread():
        refresh()
        if controller is not None and controller.debug_should_pause_at_line(line_no):
            from compiler.commands import ExecutionPaused
            raise ExecutionPaused(line_no)
