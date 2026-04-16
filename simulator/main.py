import sys
import os

# Garantiza que (1) el directorio raíz del proyecto sea el cwd para que las rutas
# relativas a assets/, buttons/ y robot_data.json funcionen, y (2) simulator/ esté
# en sys.path para que los imports como "import graphics.gui" resuelvan correctamente.
# Esto permite ejecutar el simulador desde cualquier directorio:
#   python simulator/main.py   (desde la raíz)
#   python main.py             (desde simulator/)
_SIMULATOR_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT  = os.path.dirname(_SIMULATOR_DIR)
os.chdir(_PROJECT_ROOT)
if _SIMULATOR_DIR not in sys.path:
    sys.path.insert(0, _SIMULATOR_DIR)

import graphics.gui as gui


def main():
    app = gui.MainApplication()
    app.mainloop()


if __name__ == '__main__':
    main()
