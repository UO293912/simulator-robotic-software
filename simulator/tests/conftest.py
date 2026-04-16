"""
Configuración de pytest para el módulo motor3d.
Añade el directorio simulator/ al path para que los imports funcionen.
"""
import sys
import os

# El directorio padre (simulator/) debe estar en el path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
